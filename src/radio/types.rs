use dw1000::configs::{UwbChannel, BitRate, PulseRepetitionFrequency, SfdSequence, PreambleLength};
use crate::units::{
    NanoSeconds,
    MicroSeconds,
};
use crate::config;
use crate::board::hal;
use dw1000::{DW1000, Ready, Receiving, Sending, TxConfig};
use crate::units;
use typenum::marker_traits::Unsigned;
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;
use rtic::cyccnt::Instant as CycntInstant;
use rtic::cyccnt::Duration as CycntDuration;
use dw1000::time::Instant as RadioInstant;
use core::fmt::Formatter;
use dw1000::mac::Address;
use core::fmt;

#[derive(Debug)]
pub enum Error {
    NotEnoughSpace,
    Eof,
    WrongId,
    WindowTooLong,
    WrongChannel,
    WrongBitrate,
    WrongSlotType,
    MuxTooBig,
    DemuxNotEnoughData,
    WrongDiscriminant,
}

#[cfg(feature = "pozyx-board")]
pub type Dw10000SpiIface = hal::stm32::SPI1;
#[cfg(feature = "gcharger-board")]
pub type Dw10000SpiIface = hal::stm32::SPI3;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type Dw10000SpiIface = hal::stm32::SPI1;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type Dw10000SpiIface = hal::stm32::SPI2;

pub type Dw1000Spi = hal::spi::Spi<Dw10000SpiIface,
    (config::Dw1000Clk, config::Dw1000Miso, config::Dw1000Mosi)>;

pub type ReadyRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Ready>;
pub type SendingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Sending>;
pub type ReceivingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Receiving>;

pub enum RadioState {
    Ready(Option<ReadyRadio>),

    #[cfg(feature = "master")]
    GTSStartSending((Option<SendingRadio>, RadioConfig)),
    #[cfg(feature = "master")]
    GTSAnswersReceiving((Option<ReceivingRadio>, u8, RadioConfig)),

    DynReceiving((Option<ReceivingRadio>, RadioConfig)),
    DynSending(Option<SendingRadio>),
    //DynamicWindowAckSending(Option<SendingRadio>),
    //DynamicWindowDataSending(Option<SendingRadio>),

    #[cfg(any(feature = "slave", feature = "anchor"))]
    GTSStartWaiting((Option<ReceivingRadio>, RadioConfig)),
    #[cfg(any(feature = "slave", feature = "anchor"))]
    GTSAnswerSending(Option<SendingRadio>),

    RangingPingSending((Option<SendingRadio>, CycntInstant, MicroSeconds, RadioConfig)),
    RangingPingWaiting((Option<ReceivingRadio>, CycntInstant, MicroSeconds, RadioConfig)),
    RangingRequestSending((Option<SendingRadio>, CycntInstant, MicroSeconds, RadioConfig)),
    RangingRequestWaiting((Option<ReceivingRadio>, CycntInstant, MicroSeconds, RadioConfig)),
    RangingResponseSending((Option<SendingRadio>, CycntInstant, MicroSeconds, RadioConfig)),
    RangingResponseWaiting((Option<ReceivingRadio>, CycntInstant, MicroSeconds, RadioConfig)),

    OneOffSending(Option<SendingRadio>),
    Listening(Option<ReceivingRadio>)
}
impl fmt::Debug for RadioState {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            #[cfg(feature = "master")]
            RadioState::GTSStartSending((_, _)) => { write!(f, "GTSStartSending") }
            #[cfg(feature = "master")]
            RadioState::GTSAnswersReceiving((_, _, _)) => { write!(f, "GTSAnswersReceiving") },
            RadioState::Ready(_) => { write!(f, "Ready") }
            RadioState::DynReceiving(_) => { write!(f, "DynReceiving") }
            RadioState::DynSending(_) => { write!(f, "DynSending") }
            #[cfg(any(feature = "slave", feature = "anchor"))]
            RadioState::GTSStartWaiting(_) => { write!(f, "GTSStartWaiting") }
            #[cfg(any(feature = "slave", feature = "anchor"))]
            RadioState::GTSAnswerSending(_) => { write!(f, "GTSAnswerSending") }
            RadioState::RangingPingSending(_) => { write!(f, "RangingPingSending") }
            RadioState::RangingPingWaiting(_) => { write!(f, "RangingPingWaiting") }
            RadioState::RangingRequestSending(_) => { write!(f, "RangingRequestSending") }
            RadioState::RangingRequestWaiting(_) => { write!(f, "RangingRequestWaiting") }
            RadioState::RangingResponseSending(_) => { write!(f, "RangingResponseSending") }
            RadioState::RangingResponseWaiting(_) => { write!(f, "RangingResponseWaiting") }
            RadioState::OneOffSending(_) => { write!(f, "OneOffSending") }
            RadioState::Listening(_) => { write!(f, "Listening") }
        }

    }
}

impl RadioState {
    pub fn is_sending_state(&self) -> bool {
        use RadioState::*;
        match self {
            Ready(_) => { false },
            #[cfg(feature = "master")]
            GTSStartSending(_) => { true },
            #[cfg(feature = "master")]
            GTSAnswersReceiving(_) => { false },
            #[cfg(any(feature = "slave", feature = "anchor"))]
            GTSStartWaiting(_) => { false },
            #[cfg(any(feature = "slave", feature = "anchor"))]
            GTSAnswerSending(_) => { true },
            DynReceiving(_) => { false },
            DynSending(_) => { true },
            RangingPingSending(_) | RangingRequestSending(_) | RangingResponseSending(_)  => { true },
            RangingPingWaiting(_) | RangingRequestWaiting(_) | RangingResponseWaiting(_) => { false },
            OneOffSending(_) => { true },
            Listening(_) => { false }
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RadioConfig {
    pub channel: UwbChannel,
    pub bitrate: BitRate,
    pub prf: PulseRepetitionFrequency,
}

impl Into<TxConfig> for RadioConfig {
    fn into(self) -> TxConfig {
        TxConfig {
            bitrate: self.bitrate,
            ranging_enable: false,
            pulse_repetition_frequency: self.prf,
            preamble_length: self.recommended_preamble(),
            channel: self.channel,
            sfd_sequence: self.recommended_sfd()
        }
    }
}

impl Default for RadioConfig {
    fn default() -> Self {
        RadioConfig {
            channel: config::DEFAULT_UWB_CHANNEL,
            bitrate: BitRate::Kbps850,
            prf: PulseRepetitionFrequency::Mhz64
        }
    }
}

impl RadioConfig {
    pub fn recommended_sfd(&self) -> SfdSequence {
        use SfdSequence::*;
        match self.bitrate {
            BitRate::Kbps110 =>  { Decawave },
            BitRate::Kbps850 =>  { DecawaveAlt },
            BitRate::Kbps6800 => { IEEE },
        }
    }

    pub fn recommended_preamble(&self) -> PreambleLength {
        use PreambleLength::*;
        match self.bitrate {
            BitRate::Kbps110 =>  { Symbols1024 },
            BitRate::Kbps850 =>  { Symbols256 },
            BitRate::Kbps6800 => { Symbols64 },
        }
    }

    pub fn fast() -> Self {
        RadioConfig {
            channel: RadioConfig::default().channel,
            bitrate: BitRate::Kbps6800,
            prf: PulseRepetitionFrequency::Mhz64
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SlotType {
    /// Slave sends data to master, possibly with ACK request
    GtsUplink = 0b000,
    /// Slave listents for data from master (not currently used).
    Downlink = 0b001,
    /// Everyone can send something, except master (only listens).
    Aloha = 0b010,
    /// Dynamic slot, master listens first
    DynUplink = 0b011,
    ///
    Ranging = 0b100,
    _Reserved1 = 0b101,
    _Reserved2 = 0b110,
    _Reserved3 = 0b111,
}

impl Default for SlotType {
    fn default() -> Self { SlotType::_Reserved1 }
}

/// One window of message exchanges. Requested by slaves and granted by master.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Slot {
    /// Beginning of the window from received GTSStart
    pub shift: MicroSeconds,
    /// Duration of the window
    pub duration: MicroSeconds,
    /// Who iniates transmissions in this window
    pub slot_type: SlotType,
    /// Radio config used
    pub radio_config: RadioConfig,
}

#[derive(PartialEq)]
pub enum Command {
    #[cfg(feature = "master")]
    GTSStart(RadioConfig),
    GTSEnd,
    #[cfg(any(feature = "slave", feature = "anchor"))]
    ListenForGTSStart(RadioConfig),
    #[cfg(any(feature = "slave", feature = "anchor"))]
    SendGTSAnswer(MicroSeconds, RadioConfig),
    AlohaSlotStart(MicroSeconds, RadioConfig),

    DynListen(MicroSeconds, RadioConfig),
    DynSend(MicroSeconds, RadioConfig),

    RangingStart(MicroSeconds, RadioConfig),
    ForceReady,
    ForceReadyIfSending,

    SetAntennaDelay(u16, u16), // tx, rx
    SendMessage(RadioConfig, Address, DummyMessage),
    Listen(RadioConfig),
}

#[derive(Copy, Clone, PartialEq)]
pub struct DummyMessage {
    pub length: u16,
}

#[allow(dead_code)]
pub enum Event {
    // Guaranteed slot handling
    /// scheduled: when GTSStart has been received and after power on.
    /// dt: a little bit before the next GTSStart.
    /// ▶ start listening for a GTSStart.
    #[cfg(any(feature = "slave", feature = "anchor"))]
    GTSStartAboutToBeBroadcasted,
    /// * Emitted from the SM when GTSStart message received from the PAN master.
    /// * .0 is a time when message was received, use to schedule tasks to the time mark irrelevant of processing delays
    /// * ▶ action: schedule gt, aloha and dyn slot starts and guards, if available.
    /// * ▶ action: if gt slot available with zero shift, execute GTSAboutToStart event right away.
    #[cfg(any(feature = "slave", feature = "anchor"))]
    GTSStartReceived(CycntInstant),

    /// * Emitted from the SM when GTSStart message was just sent with the current cycle counter value.
    #[cfg(feature = "master")]
    TimeMark(CycntInstant),

    /// * m: scheduled: scheduled by itself or spawned after power on. `.0` is the sum of all gt slot durations.
    /// * dt: [GTS_PERIOD](config::GTS_PERIOD)
    /// * m: ▶ issue GTSStart command to the SM, schedule itself.
    /// * s: scheduled: after GTSStart received with gt slot available if shift is not 0. `.0` is the slot duration.
    /// * dt: `slot.shift`
    /// * s: ▶ issue command to the SM to send the uplink data.
    GTSAboutToStart(MicroSeconds, RadioConfig),
    /// * m: emitted from the SM when all pending answers has been received.
    /// If some is missed, `GTSShouldHaveEnded` will disable the receivier.
    /// * s: emitted from the SM when gt slot uplink data has been sent.
    /// * ▶ set flag, notify other tasks if needed to synchronize them within all nodes in a PAN.
    GTSProcessingFinished,
    /// * on master: at the `timing marker` when all slots should be finished.
    /// * on slave: scheduled when GTSStart is received if gt slot is available, after that slot.
    /// * ▶ if not flag => force stop transmitter, not to overlap with anyone else (this is a BUG on slave!).
    GTSShouldHaveEnded,

    // Aloha slot handling
    /// * m: scheduled: when GTSStart was just sended.
    /// * s: scheduled: when GTSStart received with aloha slot available.
    /// * dt: GT phase duration + guard.
    /// * `.0` is the slot duration.
    /// * ▶ issue command to the SM to receive or transmit something.
    AlohaSlotAboutToStart(MicroSeconds, RadioConfig),
    // Can't know in advance if someone is going to transmit, wait `AlohaSlotShouldHaveEnded` event.
    //AlohaSlotProcessingFinished,
    /// * scheduled: at the `timing marker` a little bit into guard interval afterwards.
    /// * ▶ force idle state
    AlohaSlotEnded,

    // Dyn slot handling
    /// * m: scheduled: when GTSStart was just sended.
    /// * dt:
    /// * slave: scheduled after GTSStart received with dyn slot available.
    /// * ▶ action: issue command to the SM to receive or transmit something.
    DynUplinkAboutToStart(MicroSeconds, RadioConfig),
    /// * Emitted from the SM when something had been actually sent.
    /// * ▶ set flag
    DynProcessingFinished,
    /// * • Scheduled at the `timing marker` a little bit into guard interval afterwards.
    /// * ▶ if not flag => force stop transmitter, not to overlap with anyone else (this is a BUG!).
    DynShouldHaveEnded,

    // Ranging slot handling

    RangingSlotAboutToStart(MicroSeconds, RadioConfig),
    RangingSlotEnded,

    // Checking
    /// * Emitted from the SM if radio irq is pended but message was not received yet.
    /// * ▶ schedule itself and pend radio irq to re-enable receiver, if `config::DW1000_CHECK_PERIOD_MS` passed.
    /// It can get stuck, at least on RTT disconnect, and do not receive anything.
    #[cfg(any(feature = "slave", feature = "anchor"))]
    ReceiveCheck,
    // Emitted from the SM if radio irq is pended but message was not sended yet.
    //StillSending,
}

impl core::fmt::Display for Event {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(any(feature = "slave", feature = "anchor"))]
            Event::GTSStartAboutToBeBroadcasted => { write!(f, "G_ATB") },
            #[cfg(any(feature = "slave", feature = "anchor"))]
            Event::GTSStartReceived(_) => { write!(f, "G_SR") },
            #[cfg(feature = "master")]
            Event::TimeMark(_) => { write!(f, "TM") },
            Event::GTSAboutToStart(_, _) => { write!(f, "G_AS") },
            Event::GTSProcessingFinished => { write!(f, "G_PF") },
            Event::GTSShouldHaveEnded => { write!(f, "G_SX") },
            Event::AlohaSlotAboutToStart(_, _) => { write!(f, "A_ATS") },
            Event::AlohaSlotEnded => { write!(f, "A_X") },
            Event::DynUplinkAboutToStart(_, _) => { write!(f, "D_ATS") },
            Event::DynProcessingFinished => { write!(f, "Dyn_PF") },
            Event::DynShouldHaveEnded => { write!(f, "D_SX") },
            #[cfg(any(feature = "slave", feature = "anchor"))]
            Event::ReceiveCheck => { write!(f, "RC") }
            Event::RangingSlotAboutToStart(_, _) => { write!(f, "R_ATS") }
            Event::RangingSlotEnded => { write!(f, "R_SX") }
        }
    }
}

pub type CommandQueue = Queue<Command, U8>;
pub type CommandQueueP = Producer<'static, Command, U8>;
pub type CommandQueueC = Consumer<'static, Command, U8>;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Node {
    //pub address: dw1000::mac::ShortAddress,
    pub last_seen: Option<(CycntInstant, RadioInstant)>,
    pub slots: [Option<Slot>; 3],
    //#[cfg(feature = "stats")]
    // stats: NodeStatistics
}

#[allow(dead_code)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum NodeState {
    Disconnected,
    Active(Node),
}

impl NodeState {
    #[allow(dead_code)]
    pub fn find_slot(&self, t: SlotType) -> Option<Slot> {
        match self {
            NodeState::Disconnected => { None },
            NodeState::Active(node) => {
                for s in node.slots.iter() {
                    if s.is_some() {
                        if s.unwrap().slot_type == t {
                            return *s;
                        }
                    }
                }
                None
            }
        }
    }
}

pub struct Radio {
    pub(crate) state: RadioState,
    pub(crate) state_instant: Option<CycntInstant>,
    pub irq: config::RadioIrqPin,
    pub(crate) commands: CommandQueueC,
    // #[cfg(any(feature = "master", feature = "devnode"))]
    // pub(crate) nodes: [NodeState; config::TotalNodeCount::USIZE],
    #[cfg(any(feature = "slave", feature = "anchor"))]
    pub(crate) master: NodeState,
    //address_map: heapless::FnvIndexMap<dw1000::mac::ShortAddress, usize, config::TOTAL_NODE_COUNT>,
}

impl Radio {
    pub fn new(dw1000: ReadyRadio, irq: config::RadioIrqPin, commands: CommandQueueC) -> Self {
        Radio {
            state: RadioState::Ready(Some(dw1000)),
            state_instant: None,
            irq,
            commands,
            // #[cfg(any(feature = "master", feature = "devnode"))]
            // nodes: [NodeState::Disconnected; config::TotalNodeCount::USIZE],
            #[cfg(any(feature = "slave", feature = "anchor"))]
            master: NodeState::Disconnected
        }
    }
}

#[derive(Default)]
pub struct Stat {
    pub tr_gts_answers: u32,
    pub bl_gts_answers: u32,
    pub br_gts_answers: u32,
}

pub struct Pong {

}
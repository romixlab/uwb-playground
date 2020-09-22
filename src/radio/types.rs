use dw1000::configs::{UwbChannel, BitRate, PulseRepetitionFrequency, SfdSequence, PreambleLength};
use crate::units::{
    NanoSeconds,
    MicroSeconds,
};
use crate::config;
use crate::board::hal;
use dw1000::{
    DW1000,
    Ready,
    Receiving,
    Sending
};
use crate::units;
use typenum::marker_traits::Unsigned;
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;
use rtic::cyccnt::Instant as CycntInstant;
use rtic::cyccnt::Duration as CycntDuration;
use dw1000::time::Instant as RadioInstant;
use core::fmt::Formatter;

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

pub type Dw1000Spi = hal::spi::Spi<hal::stm32::SPI1,
    (config::Dw1000Clk, config::Dw1000Miso, config::Dw1000Mosi)>;

pub type ReadyRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Ready>;
pub type SendingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Sending>;
pub type ReceivingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Receiving>;

pub enum RadioState {
    Ready(Option<ReadyRadio>),

    #[cfg(feature = "master")]
    GTSStartSending(Option<SendingRadio>),
    #[cfg(feature = "master")]
    GTSAnswersReceiving((Option<ReceivingRadio>, u8)),

    DynReceiving((Option<ReceivingRadio>, RadioConfig)),
    DynSending(Option<SendingRadio>),
    //DynamicWindowAckSending(Option<SendingRadio>),
    //DynamicWindowDataSending(Option<SendingRadio>),

    #[cfg(feature = "slave")]
    GTSStartWaiting(Option<ReceivingRadio>),
    #[cfg(feature = "slave")]
    GTSAnswerSending(Option<SendingRadio>),
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
            #[cfg(feature = "slave")]
            GTSStartWaiting(_) => { false },
            #[cfg(feature = "slave")]
            GTSAnswerSending(_) => { true },
            DynReceiving(_) => { false },
            DynSending(_) => { true },
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RadioConfig {
    pub channel: UwbChannel,
    pub bitrate: BitRate,
    pub prf: PulseRepetitionFrequency,
}

impl Default for RadioConfig {
    fn default() -> Self {
        RadioConfig {
            channel: UwbChannel::Channel5,
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
            channel: UwbChannel::Channel5,
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
    Reserved = 0b111,
}

impl Default for SlotType {
    fn default() -> Self { SlotType::Reserved }
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
    GTSStart,
    GTSEnd,
    #[cfg(feature = "slave")]
    ListenForGTSStart(RadioConfig),
    #[cfg(feature = "slave")]
    SendGTSAnswer(MicroSeconds, RadioConfig),
    AlohaSlotStart(MicroSeconds, RadioConfig),

    DynListen(MicroSeconds, RadioConfig),
    DynSend(MicroSeconds, RadioConfig),

    RangingStart(MicroSeconds, RadioConfig),
    ForceReady,
    ForceReadyIfSending,
}

pub enum Event {
    // Guaranteed slot handling
    /// scheduled: when GTSStart has been received and after power on.
    /// dt: a little bit before the next GTSStart.
    /// ▶ start listening for a GTSStart.
    #[cfg(feature = "slave")]
    GTSStartAboutToBeBroadcasted,
    /// * Emitted from the SM when GTSStart message received from the PAN master.
    /// * .0 is a time when message was received, use to schedule tasks to the time mark irrelevant of processing delays
    /// * ▶ action: schedule gt, aloha and dyn slot starts and guards, if available.
    /// * ▶ action: if gt slot available with zero shift, execute GTSAboutToStart event right away.
    #[cfg(feature = "slave")]
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
    #[cfg(feature = "slave")]
    ReceiveCheck,
    // Emitted from the SM if radio irq is pended but message was not sended yet.
    //StillSending,
}

impl core::fmt::Display for Event {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "slave")]
            Event::GTSStartAboutToBeBroadcasted => { write!(f, "G_ATB") },
            #[cfg(feature = "slave")]
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
            #[cfg(feature = "slave")]
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

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum NodeState {
    Disconnected,
    Active(Node),
}

impl NodeState {
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
    #[cfg(any(feature = "master", feature = "devnode"))]
    pub(crate) nodes: [NodeState; config::TotalNodeCount::USIZE],
    #[cfg(feature = "slave")]
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
            #[cfg(any(feature = "master", feature = "devnode"))]
            nodes: [NodeState::Disconnected; config::TotalNodeCount::USIZE],
            #[cfg(feature = "slave")]
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
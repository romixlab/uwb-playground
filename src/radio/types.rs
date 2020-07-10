use dw1000::configs::{
    UwbChannel,
    BitRate,
    PulseRepetitionFrequency,
};
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

#[derive(Debug)]
pub enum Error {
    NotEnoughSpace,
    Eof,
    WrongId,
    WindowTooLong,
    WrongChannel,
    WrongBitrate,
    MuxTooBig,
    DemuxNotEnoughData,
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

    //DynamicWindowReceiving((Option<ReceivingRadio>)),
    //DynamicWindowAckSending(Option<SendingRadio>),
    //DynamicWindowDataSending(Option<SendingRadio>),

    #[cfg(feature = "slave")]
    GTSStartWaiting(Option<ReceivingRadio>),
    #[cfg(feature = "slave")]
    GTSWaitingForUplinkData(Option<ReadyRadio>),
    #[cfg(feature = "slave")]
    GTSAnswerSending(Option<SendingRadio>),
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RadioConfig {
    pub channel: UwbChannel,
    pub bitrate: BitRate,
    pub prf: PulseRepetitionFrequency,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WindowType {
    /// Slave sends data to master, possibly with ACK request
    Uplink = 0b0,
    /// Slave listents for data from master
    Downlink = 0b1
}

/// One window of message exchanges. Requested by slaves and granted by master.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Window {
    /// Beginning of the window from received GTSStart
    pub shift: MicroSeconds,
    /// Duration of the window
    pub window: MicroSeconds,
    /// Who iniates transmissions in this window
    pub window_type: WindowType,
    /// Radio config used
    pub radio_config: RadioConfig,
}

pub enum Command {
    #[cfg(feature = "master")]
    GTSStart,
    #[cfg(feature = "master")]
    GTSEnd,
}

#[derive(Debug)]
pub enum Event {
    //#[cfg(any(feature = "master", feature = "devnode"))]
    //GTSAnswerReceived(dw1000::mac::ShortAddress, message::GTSAnswer),
    #[cfg(feature = "slave")]
    /// .0 - tx_time in dw1000 counts; .1 - time till GTS end.
    GTSStartReceived(Option<dw1000::time::Instant>, NanoSeconds),
    GTSEnded,
}

pub type CommandQueue = Queue<Command, U8>;
pub type CommandQueueP = Producer<'static, Command, U8>;
pub type CommandQueueC = Consumer<'static, Command, U8>;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Node {
    pub address: dw1000::mac::ShortAddress,
    pub last_seen: units::MilliSeconds,
    pub slots: [Option<Window>; 2],
    //#[cfg(feature = "stats")]
    // stats: NodeStatistics
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum NodeState {
    Disconnected,
    Active(Node),
}

pub struct Radio {
    pub(crate) state: RadioState,
    pub irq: config::RadioIrqPin,
    pub(crate) commands: CommandQueueC,
    #[cfg(any(feature = "master", feature = "devnode"))]
    nodes: [NodeState; config::TotalNodeCount::USIZE],
    #[cfg(feature = "slave")]
    master: NodeState,
    //address_map: heapless::FnvIndexMap<dw1000::mac::ShortAddress, usize, config::TOTAL_NODE_COUNT>,
}

impl Radio {
    pub fn new(dw1000: ReadyRadio, irq: config::RadioIrqPin, commands: CommandQueueC) -> Self {
        Radio {
            state: RadioState::Ready(Some(dw1000)),
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
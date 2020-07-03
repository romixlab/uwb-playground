pub mod message;
pub mod state_machine;

use dw1000::{DW1000, Ready, Receiving, Sending};
use crate::config;
use crate::board::hal;

use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;

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

    #[cfg(feature = "slave")]
    GTSStartWaiting(Option<ReceivingRadio>),
    #[cfg(feature = "slave")]
    GTSWaitingForUplinkData(Option<ReadyRadio>),
    #[cfg(feature = "slave")]
    GTSAnswerSending(Option<SendingRadio>),
}

pub enum Command {
    #[cfg(feature = "master")]
    GTSStart(message::GTSStart),
    /// Fired when all GTS should finish on master and slave, after that rpms are sent to MCs.
    #[cfg(feature = "slave")]
    GTSSendAnswer(Option<dw1000::time::Instant>, message::GTSAnswer),
    GTSEnd,
}

#[derive(Debug, PartialEq, Copy, Clone,)]
pub struct NanoSeconds(pub u32);

#[derive(Debug)]
pub enum Event {
    #[cfg(any(feature = "master", feature = "devnode"))]
    GTSAnswerReceived(dw1000::mac::ShortAddress, message::GTSAnswer),
    #[cfg(feature = "slave")]
    /// .0 - tx_time in dw1000 counts; .1 - time till GTS end.
    GTSStartReceived(Option<dw1000::time::Instant>, NanoSeconds, message::GTSEntry),
    GTSEnded,
}

pub type CommandQueue = Queue<Command, U8>;
pub type CommandQueueP = Producer<'static, Command, U8>;
pub type CommandQueueC = Consumer<'static, Command, U8>;

/// Lost -> smth received -> Active(0)
/// slot missed -> Active(+1), threshold reached -> Lost


struct Window {
    shift: u16,
    window: u16
}

use crate::units::MilliSeconds;
struct Node {
    address: dw1000::mac::ShortAddress,
    last_seen: MilliSeconds,
    granted_window: Option<Window>
    //#[cfg(feature = "stats")]
    // stats: NodeStatistics
}

pub enum NodeState {
    Disconnected,
    Active(Node),
}

use typenum::marker_traits::Unsigned;
struct Radio {
    nodes: [NodeState; config::TOTAL_NODE_COUNT::USIZE],
    //address_map: heapless::FnvIndexMap<dw1000::mac::ShortAddress, usize, config::TOTAL_NODE_COUNT>,
}
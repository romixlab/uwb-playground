pub mod message;
pub mod state_machine;
pub mod serdes;

use crate::units;

use dw1000::{DW1000, Ready, Receiving, Sending};
use crate::config;
use crate::board::hal;

use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;

#[derive(Debug)]
pub enum Error {
    WindowTooLong,
    WrongChannel,
    WrongBitrate,
    MuxTooBig,
    DemuxNotEnoughData,
}

/// Logical destination inside a multiplexed message.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum LogicalDestination {
    /// Assume that multiplexed message destination is Self upon receiving.
    /// Useful when slave sends a message to master, or master sends a unicast message to specific node.
    Implicit,
    /// Send to specified node address.
    /// Useful when master sends multicast multiplexed message for many nodes.
    #[cfg(any(feature = "master", feature = "devnode", feature = "relay"))]
    Unicast(dw1000::mac::ShortAddress),
}

/// Multiplexed channel id
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct ChannelId(u8);

impl ChannelId {
    pub fn new(channel_id: u8) -> Self {
        assert!(channel_id <= 127, "ChannelId must be <= 127");
        ChannelId(channel_id)
    }

    pub fn id(&self) -> u8 { self.0 }
}

/// Multiplex several messages into one continous stream of bytes.
pub trait Multiplexer {
    type Error;

    fn mux(&mut self,
           thing: &dyn serdes::Serialize<Error=Self::Error>,
           destination: LogicalDestination,
           channel: ChannelId
    ) -> Result<(), Self::Error>;
}

/// Interface between the radio and data sources and sinks.
pub trait Arbiter {
    /// Called each GTS to get some data to send. Number of bytes that may be sent is calculated
    /// from slot duration, transmitter settings, spi & cpu core speed.
    /// Return:
    /// * .0 - Actual number of bytes written.
    /// * .1 - Logical destination (all slaves or just one of them (for master)). Message will be
    ///     received by all nodes in a pan, parts of it will be ignored accordingly.
    ///     Destination is ignored for slaves for now (slave->slave gts data is not needed?).
    /// * .2 - Channel ID (destination queue / buffer id).
    /// Will be called again if there is space available. If 0 is returned, no further calls will be
    /// made in a given slot.
    fn source_sync<M: Multiplexer>(&mut self, multiplexer: &mut M);
    /// Called whenever there is time to send more data (in additionaly requested slot).
    /// Semantics are the same as in `source_sync`.
    fn source_async<M: Multiplexer>(&mut self, multiplexer: &mut M);
    // Called when request for additional time slot may be made. Return current amount of data
    // currently pending for transmission
    //fn size_hints(&self) ->
    /// Called when data for a given channel had been received in async slot.
    fn sink_sync(&mut self, buf: &[u8], channel: ChannelId);
    /// Called when data for a given channel had been received in sync slot.
    fn sink_async(&mut self, buf: &[u8], channel: ChannelId);
}

/// Simple multiplexer that uses from 2 to 5 bytes for each muxed message.
/// byte 0: DCCC_CCCC, where C - ChannelId <= 127, D = 1 if logical address is in the next 2 bytes.
/// bytes +1,2: logical address or Implicit if D = 0.
/// bytes +1: MUUU_UUUU, where U - message length if <= 127, M - more flag.
/// bytes +1 if M: UUUU_UUUU - additional high bits for length.
pub struct MiniMultiplexer<'a> {
    buf: &'a mut [u8],
    cursor: usize
}

impl<'a> MiniMultiplexer<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        MiniMultiplexer {
            buf,
            cursor: 0
        }
    }

    fn header_size(message_len: usize, destination: LogicalDestination, channel: ChannelId) -> usize
    {
        let mut needed_for_header = if destination == LogicalDestination::Implicit { 1 } else { 3 };
        needed_for_header += if message_len <= 127 { 1 } else { 2 };
        needed_for_header
    }

    #[inline]
    fn remaining(&self) -> usize {
        self.buf.len() - self.cursor
    }

    pub fn written(&self) -> usize {
        self.cursor
    }
}

impl<'a> Multiplexer for MiniMultiplexer<'a> {
    type Error = Error;

    fn mux(
        &mut self,
        thing: &dyn serdes::Serialize<Error=Self::Error>,
        destination: LogicalDestination,
        channel: ChannelId
    ) -> Result<(), Error>
    {
        let header_size = MiniMultiplexer::header_size(thing.size_hint(), destination, channel);
        let thing_size = thing.size_hint();
        if header_size + thing_size > self.remaining() || thing_size >= 32768 {
            return Err(Error::MuxTooBig);
        }
        if let LogicalDestination::Unicast(addr) = destination {
            self.buf[self.cursor] = 0b1000_0000 | channel.id();
            self.buf[self.cursor+1 ..= self.cursor + 2].copy_from_slice(&addr.0.to_le_bytes());
            self.cursor += 3;
        } else {
            self.buf[self.cursor] = channel.id();
            self.cursor += 1;
        }
        if thing_size <= 127 {
            self.buf[self.cursor] = thing_size as u8;
            self.cursor += 1;
        } else {
            let len = thing_size as u16;
            self.buf[self.cursor] = 0b1000_0000 | (len & 0b0111_1111) as u8;
            self.buf[self.cursor + 1] = (len >> 9) as u8;
            self.cursor += 2;
        }
        thing.ser(&mut self.buf[self.cursor..])?;
        self.cursor += thing_size;
        Ok(())
    }
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

    #[cfg(feature = "slave")]
    GTSStartWaiting(Option<ReceivingRadio>),
    #[cfg(feature = "slave")]
    GTSWaitingForUplinkData(Option<ReadyRadio>),
    #[cfg(feature = "slave")]
    GTSAnswerSending(Option<SendingRadio>),
}

pub enum Command {
    #[cfg(feature = "master")]
    GTSStart,
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

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Node {
    pub address: dw1000::mac::ShortAddress,
    pub last_seen: units::MilliSeconds,
    //pub slots: [Option<Window>; 2]
    //#[cfg(feature = "stats")]
    // stats: NodeStatistics
}

pub enum NodeState {
    Disconnected,
    Active(Node),
}

use typenum::marker_traits::Unsigned;
use core::convert::TryInto;

struct Radio {
    hw: RadioState,
    #[cfg(any(feature = "master", feature = "devnode"))]
    nodes: [NodeState; config::TotalNodeCount::USIZE],
    #[cfg(feature = "slave")]
    master: NodeState,
    //address_map: heapless::FnvIndexMap<dw1000::mac::ShortAddress, usize, config::TOTAL_NODE_COUNT>,
}

#[derive(Default)]
pub struct Stat {
    pub tr_gts_answers: u32,
    pub bl_gts_answers: u32,
    pub br_gts_answers: u32,

}

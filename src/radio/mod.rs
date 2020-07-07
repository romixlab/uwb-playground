pub mod message;
pub mod state_machine;

use crate::units;

use dw1000::{DW1000, Ready, Receiving, Sending};
use crate::config;
use crate::board::hal;

use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;

use serde::{
    Serialize, Deserialize
};

/// Logical destination inside GTSStart message.
pub enum LogicalDestination {
    /// Send to specified node address
    Unicast(dw1000::mac::ShortAddress),
    /// Send to all members of a PAN
    Multicast,
}

pub struct BytesWritten(pub u16);
pub struct ChannelId(pub u8);

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
    fn source_sync(&mut self, buf: &mut [u8]) -> (BytesWritten, LogicalDestination, ChannelId);
    /// Called whenever there is time to send more data (in additionaly requested slot).
    /// Semantics are the same as in `source_sync`.
    fn source_async(&mut self, buf: &mut [u8], /*confidence: */) -> (BytesWritten, LogicalDestination, ChannelId);
    // Called when request for additional time slot may be made. Return current amount of data
    // currently pending for transmission
    //fn size_hints(&self) ->
    /// Called when data for a given channel had been received in sync slot.
    fn sink_async(&mut self, buf: &[u8], channel: ChannelId);
    /// Called when data for a given channel had been received in async slot.
    fn sink_sync(&mut self, buf: &[u8], channel: ChannelId);
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

pub trait MessageId {
    const ID: u32;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RadioConfig {
    pub channel: dw1000::configs::UwbChannel,
    pub bitrate: dw1000::configs::BitRate,
    pub prf: dw1000::configs::PulseRepetitionFrequency,
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
    pub shift: units::MicroSeconds,
    /// Duration of the window
    pub window: units::MicroSeconds,
    /// Who iniates transmissions in this window
    pub window_type: WindowType,
    /// Radio config used
    pub radio_config: RadioConfig,
}

pub enum Error {
    WindowTooBig,
    WrongChannel,
    WrongBitrate,
}

fn channel_from_u8(n: u8) -> Option<dw1000::configs::UwbChannel> {
    use dw1000::configs::UwbChannel::*;
    match n {
        1 => { Some(Channel1) },
        2 => { Some(Channel2) },
        3 => { Some(Channel3) },
        4 => { Some(Channel4) },
        5 => { Some(Channel5) },
        7 => { Some(Channel7) },
        _ => None
    }
}

fn bitrate_from_u8(n: u8) -> Option<dw1000::configs::BitRate> {
    use dw1000::configs::BitRate::*;
    match n {
        0b00 => { Some(Kbps110) },
        0b01 => { Some(Kbps850) },
        0b10 => { Some(Kbps6800) },
        _ => None
    }
}

impl Window {
    pub fn serialize(&self, buf: &mut[u8; 5]) -> Result<(), Error> {
        if self.shift.0 >= core::u16::MAX as u32 || self.window.0 >= core::u16::MAX as u32 {
            return Err(Error::WindowTooBig);
        }
        let shift = self.shift.0 as u16;
        let window = self.window.0 as u16;
        buf[0..=1].copy_from_slice(&shift.to_le_bytes());
        buf[2..=3].copy_from_slice(&window.to_le_bytes());
        let channel = self.radio_config.channel as u8;
        let bitrate = self.radio_config.bitrate as u8;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if self.radio_config.prf == Mhz16 { 0u8 } else { 1u8 };
        let window_type = self.window_type as u8;
        buf[4] = (window_type << 7) | (prf << 6) | (bitrate << 4) | channel;
        Ok(())
    }

    pub fn deserizalize(buf: &[u8; 5]) -> Result<Self, Error> {
        let shift: [u8; 2] = buf[0..=1].try_into().unwrap();
        let shift = u16::from_le_bytes(shift);
        let window: [u8; 2] = buf[2..=3].try_into().unwrap();
        let window = u16::from_le_bytes(window);
        let channel = channel_from_u8(buf[4] & 0b0000_1111).ok_or(Error::WrongChannel)?;
        let bitrate = bitrate_from_u8((buf[4] & 0b0011_0000) >> 4).ok_or(Error::WrongBitrate)?;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if buf[4] & 0b0100_0000 == 0 { Mhz16 } else { Mhz64 };
        use WindowType::*;
        let window_type = if buf[4] & 0b1000_0000 == 0 { Uplink } else { Downlink };
        Ok(Window {
            shift: units::MicroSeconds(shift as u32),
            window: units::MicroSeconds(window as u32),
            window_type,
            radio_config: RadioConfig {
                channel,
                bitrate,
                prf
            }
        })
    }
}

impl MessageId for Window {
    const ID: u32 = 0x70;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Node {
    pub address: dw1000::mac::ShortAddress,
    pub last_seen: units::MilliSeconds,
    pub slots: [Option<Window>; 2]
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

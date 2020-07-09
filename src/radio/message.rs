//use core::mem::size_of;
use serde::{Serialize, Deserialize, Serializer};
use ssmarshal;
use dw1000::{
    mac,
    time::Instant,
    ranging::{
        Message, Prelude
    },
    DW1000, Ready, Sending, TxConfig
};
use embedded_hal::{
    blocking::spi,
    digital::v2::OutputPin,
};
use crate::units;
use super::Error;

/// Any message size is <= than this number (checked in send()).
#[allow(unused)] // Actually used, lint bug?
const MAXIMUM_MESSAGE_SIZE: usize = 64;

/// An outgoing message (dw1000 incoming implementation is reused as is)
///
/// Contains the payload to be sent, as well as some metadata.
/// Compared to dw1000 implementation, tx_time is optional and TxConfig may be passed to send.
/// Also const maximum message size is configured here.
#[derive(Debug)]
pub struct TxMessage<T: Message> {
    /// The recipient of the message
    ///
    /// This is an IEEE 802.15.4 MAC address. This could be a broadcast address,
    /// for messages that are sent to all other nodes in range.
    pub recipient: mac::Address,

    /// The time this message is going to be sent
    ///
    /// When creating this struct, this is going to be an instant in the near
    /// future. When sending the message, the sending is delayed to make sure it
    /// it sent at exactly this instant.
    pub tx_time: Option<Instant>,

    /// The actual message payload
    pub payload: T,
}

impl<T> TxMessage<T> where T: Message {
    /// Send this message via the DW1000
    ///
    /// Serializes the message payload and uses [`DW1000::send`] internally to
    /// send it.
    pub fn send<'r, SPI, CS>(&self, dw1000: DW1000<SPI, CS, Ready>, tx_config: TxConfig)
                             -> Result<DW1000<SPI, CS, Sending>, dw1000::Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        // Create a buffer that fits the biggest message currently implemented.
        // This is a really ugly hack. The size of the buffer should just be
        // `T::LEN`. Unfortunately that's not possible. See:
        // https://github.com/rust-lang/rust/issues/42863
        assert!(T::LEN <= MAXIMUM_MESSAGE_SIZE);
        let mut buf = [0; MAXIMUM_MESSAGE_SIZE];

        buf[..T::PRELUDE.0.len()].copy_from_slice(T::PRELUDE.0);
        ssmarshal::serialize(
            &mut buf[T::PRELUDE.0.len()..],
            &self.payload,
        )?;

        let future = dw1000.send(
            &buf[..T::LEN],
            self.recipient,
            self.tx_time,
            tx_config
        )?;

        Ok(future)
    }
}

/// Data sent to each slave from master every GTS phase.
#[derive(Debug, Deserialize, Serialize, Copy, Clone)]
#[repr(C)]
pub struct GTSDownlinkData {
    pub rpm: i32
}

/// Data sent to master from each slave during GTS phase.
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct GTSUplinkData {
    pub tacho: i32,
    pub power_in: f32,
}

/// Guaranteed time slot for each slave.
#[derive(Debug, Deserialize, Serialize, Copy, Clone)]
#[repr(C)]
pub struct GTSEntry {
    /// Delay in us from GTS packet reception until slave starts sending.
    pub delta: u16,
    /// Time slot duration allocated, one ore more packets can be sent during this interval.
    pub window: u16,
    /// Multicast data to all slaves, no retransmissions for this.
    pub sync_no_ack_data: GTSDownlinkData,
}

impl GTSEntry {
    #[cfg(any(feature = "master", feature = "devnode"))]
    pub fn new(delta: u16, window: u16, sync_no_ack_data: GTSDownlinkData) -> Self {
        GTSEntry {
            delta, window, sync_no_ack_data
        }
    }
}

/// Packet with GTS for all synchronous slaves.
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct GTSStart {
    pub timeslots: [GTSEntry; 3]
}

impl GTSStart {
    #[cfg(any(feature = "master", feature = "devnode"))]
    pub fn new(tr: GTSEntry, bl: GTSEntry, br: GTSEntry) -> Self {
        GTSStart {
            timeslots: [tr, bl, br]
        }
    }
}

impl Message for GTSStart {
    const PRELUDE: Prelude = Prelude(b"GTSS");
    const PRELUDE_LEN: usize = 4;
}

/// Packet from slave with it's data.
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct GTSAnswer {
    pub data: GTSUplinkData
}

impl Message for GTSAnswer {
    const PRELUDE: Prelude = Prelude(b"GTSA");
    const PRELUDE_LEN: usize = 4;
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

use crate::radio::serdes::{MessageId, Buf, BufMut};

impl super::serdes::MessageId for Window {
    const ID: u8 = 0x70;
}

impl super::serdes::Serialize for Window {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if self.shift.0 >= core::u16::MAX as u32 || self.window.0 >= core::u16::MAX as u32 {
            return Err(Error::WindowTooLong);
        }
        buf.put_u8(Self::ID);
        let shift = self.shift.0 as u16;
        let window = self.window.0 as u16;
        buf.put_u16(shift);
        buf.put_u16(window);
        let channel = self.radio_config.channel as u8;
        let bitrate = self.radio_config.bitrate as u8;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if self.radio_config.prf == Mhz16 { 0u8 } else { 1u8 };
        let window_type = self.window_type as u8;
        buf.put_u8((window_type << 7) | (prf << 6) | (bitrate << 4) | channel);
        Ok(())
    }

    fn size_hint(&self) -> usize { 6 }
}

impl super::serdes::Deserialize for Window {
    type Output = Window;
    type Error = super::Error;

    fn des(buf: &[u8]) -> Result<Self::Output, Self::Error> {
        use core::convert::TryInto;

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
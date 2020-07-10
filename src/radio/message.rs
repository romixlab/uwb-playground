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




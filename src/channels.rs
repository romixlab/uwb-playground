use crate::radio::channelization::{
    Arbiter,
    LogicalDestination,
    ChannelId,
    Multiplex,
};
use crate::radio::serdes::{
    Deserialize,
    Buf,
};

use rtt_target::{
    rprint,
    rprintln
};

use crate::config;
use cfg_if::cfg_if;

use crate::units::Watts;
use dw1000::mac::Address;
use crate::color;

/// **Reliability**
/// * `R` - (Reliable) ACKed, with retransmission (as in TCP).
/// * `N` - NACKed, with retransmission (as in CoAP).
/// * `I` - Ignore lost bytes/frames (as in UDP).
///
/// **Queueing**
/// * `0` - Unbuffered.
/// * `P` - Packetized queue with fixed or dynamic frames.
/// * `B` - Byte buffer or Stream.
///
/// **Ordering**
/// * `L` - LIFO (need newest data).
/// * `F` - FIFO (need sequential data).
///
/// **Timing**
/// * `A` - Asynchronous, sent some time in the future.
/// * `T` - Timed / synchronous, sent in a predetermined window.
///
/// **Overflow behaviour**
/// * `D` - Drop on overflow.
/// * `G` - Stop on overflow (gave up).
/// * `P` - Panic on overflow.
///
/// **Security**
/// * `E` - Encrypted.
/// * `H` - Hackable.
///
/// **Duplicates**
/// * `S` - Sequence numbered, repeated data dropped (copies may arrive as in UDP or redundant systems).
/// * `U` - Unique, channel does not create copies.
///
/// **Priority**
/// * `1` and up, default is 1. Equal priorities are round-robin scheduled. Lowest priority sync is higher than any other async.
///
/// **Channels**
/// * `@ID` - data is grabbed from specific queue and enqueued to the same id on the other node.
pub struct Channels {

}



impl Channels {


}

impl Arbiter for Channels {
    type Error = crate::radio::Error;

    fn source_sync<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M)
    {

    }

    fn source_async<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {

    }

    fn sink_sync(&mut self, source: Address,  channel: ChannelId, chunk: &[u8]) {

    }

    fn sink_async(&mut self, source: Address, channel: ChannelId, chunk: &[u8]) {

    }
}
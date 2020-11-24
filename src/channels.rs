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
    pub can0_forward_heap: crate::tasks::canbus::ForwardHeap,
}

impl Channels {
    pub fn new() -> Self {
        Channels {
            can0_forward_heap: heapless::BinaryHeap::new()
        }
    }
}

use crate::tasks::canbus;

impl Arbiter for Channels {
    type Error = crate::radio::Error;

    fn source_sync<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {
        cfg_if! {
            if #[cfg(feature = "master")] {
                let mut frames_muxed = 0;
                const MAX_FRAMES_PER_GTS: u32 = 10;
                while let Some(forward_entry) = self.can0_forward_heap.pop() {
                    let destination = match forward_entry.to {
                        canbus::Destination::Unicast(address) => LogicalDestination::Unicast(address),
                        canbus::Destination::Multicast(_) => LogicalDestination::Implicit,
                        _ => { continue; }
                    };
                    mux.mux(&forward_entry, destination, ChannelId::new(7));
                    frames_muxed += 1;
                    if frames_muxed >= MAX_FRAMES_PER_GTS {
                        break;
                    }
                }
            }
        }
    }

    fn source_async<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {

    }

    fn sink_sync(&mut self, source: Address,  channel: ChannelId, chunk: &[u8]) {

    }

    fn sink_async(&mut self, source: Address, channel: ChannelId, chunk: &[u8]) {

    }
}
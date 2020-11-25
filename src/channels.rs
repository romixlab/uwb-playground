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
    pub can0_send_heap: config::CanSendHeap,

    pub can2uwb: u32,
    pub uwb2can_ok: u32,
    pub uwb2can_drop: u32,
}

impl Channels {
    pub fn new() -> Self {
        Channels {
            can0_forward_heap: heapless::BinaryHeap::new(),
            can0_send_heap: vhrdcan::FrameHeap::new(),

            can2uwb: 0,
            uwb2can_ok: 0,
            uwb2can_drop: 0
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
                    self.can2uwb += 1;
                    frames_muxed += 1;
                    if frames_muxed >= MAX_FRAMES_PER_GTS {
                        break;
                    }
                }
            }
        }
    }

    fn source_async<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {
        cfg_if! {
            if #[cfg(any(feature = "tr", feature = "bl"))] {
                const MAX_FRAMES_PER_GTS: u32 = 10;
            } else if #[cfg(feature = "br")] {
                const MAX_FRAMES_PER_GTS: u32 = 30;
            }
        }
        cfg_if! {
            if #[cfg(feature = "slave")] {
                let mut frames_muxed = 0;
                while let Some(forward_entry) = self.can0_forward_heap.pop() {
                    let destination = match forward_entry.to {
                        canbus::Destination::Unicast(address) => LogicalDestination::Unicast(address),
                        canbus::Destination::Multicast(_) => LogicalDestination::Implicit,
                        _ => { continue; }
                    };
                    mux.mux(&forward_entry, destination, ChannelId::new(7));
                    frames_muxed += 1;
                    self.can2uwb += 1;
                    if frames_muxed >= MAX_FRAMES_PER_GTS {
                        break;
                    }
                }
            }
        }
    }

    fn sink_sync(&mut self, source: Address,  channel: ChannelId, chunk: &[u8]) {
        let mut buf = Buf::new(chunk);
        // rprintln!(=>3, "chunk:{}", chunk.len());
        // for b in chunk {
        //     rprint!(=>3, "{:02x} ", *b);
        // }
        // rprintln!(=>3, "");
        match crate::tasks::canbus::ForwardEntry::des(&mut buf) {
            Ok(raw_frame) => {
                let frame = self.can0_send_heap.pool.new_frame(raw_frame.id, raw_frame.data()).unwrap();
                match self.can0_send_heap.heap.push(frame) {
                    Ok(_) => {
                        self.uwb2can_ok += 1;
                    },
                    Err(_) => {
                        self.uwb2can_drop += 1;
                    }
                }
            },
            Err(e) => {
                rprintln!(=>3, "demux err:{:?}", e);
            }
        }
    }

    fn sink_async(&mut self, source: Address, channel: ChannelId, chunk: &[u8]) {
        self.sink_sync(source, channel, chunk);
    }
}
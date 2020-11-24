use crate::{
    config
};
use rtt_target::{rprint, rprintln};
use vhrdcan::{FrameId, Frame};
use dw1000::mac;
use cfg_if::cfg_if;
use vhrdcan::id::{StandardId, ExtendedId};
use core::cmp::Ordering;
use rtic::Mutex;

#[derive(Default)]
pub struct DirectionStatistics {
    pub frames_processed: u32,
    pub frames_dropped: u32,
    pub bytes_processed: u32
}

#[derive(Default)]
pub struct LLStatistics {
    pub rx: DirectionStatistics,
    pub tx: DirectionStatistics
}

pub fn can0_irq0(mut cx: crate::can0_irq0::Context) {
    let can: &mut config::Can0 = cx.resources.can0;
    use crate::hal::can::ClassicalCan;
    let ll_statistics: &mut LLStatistics = cx.resources.can0_ll_statistics;

    rprintln!("can_irq0");
    rprintln!("reason: {:?}", can.interrupt_reason());
    // rprintln!("{:?}", can.protocol_status());
    // rprintln!("rec:{} tec:{}", can.receive_error_counter(), can.transmit_error_counter());
    // rprintln!("rx_pin: {:?}", can.rx_pin_state());
    // rprintln!("queue: {}", cx.resources.can0_send_heap.heap.len());
    if can.free_slots() != 0 {
        if let Some(frame) = cx.resources.can0_send_heap.heap.pop() {
            match can.send(&frame) {
                Ok(_) => {
                    ll_statistics.tx.frames_processed += 1;
                    ll_statistics.tx.bytes_processed += frame.len() as u32;
                },
                Err(_) => {
                    ll_statistics.tx.frames_dropped += 1;
                }
            }

            //rprintln!("send: {:?}{:?}{}", frame.id(), r, cx.resources.can0_send_heap.heap.len());
        }
    }

    let receive_heap: &mut config::CanReceiveHeap = cx.resources.can0_receive_heap;
    can.get_all(|id, data| {
        // rprintln!("rx: {:?}, {}", id, data.len());
        // for b in data {
        //     rprint!("{:02x} ", *b);
        // }
        // rprintln!("");
        match receive_heap.pool.new_frame(id, data) {
            Ok(frame) => {
                let len = frame.len() as u32;
                match receive_heap.heap.push(frame) {
                    Ok(_) => {
                        ll_statistics.rx.frames_processed += 1;
                        ll_statistics.rx.bytes_processed += len;
                    },
                    Err(_) => {
                        ll_statistics.rx.frames_dropped += 1;
                    }
                }
            },
            Err(_) => {
                ll_statistics.rx.frames_dropped += 1;
            }
        }
    });

    unsafe { can.regs_mut().ir.write(|w| w.bits(0x00ff_ffff)) };
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Destination {
    Unicast(mac::ShortAddress),
    Multicast(mac::PanId),
    Broadcast
}

#[derive(Copy, Clone)]
pub enum RoutingAction {
    Drop,
    ProcessLocally,
    Forward(Destination),
}

#[derive(Default)]
pub struct RxRoutingStatistics {
    pub dropped: DirectionStatistics,
    pub processed_locally: DirectionStatistics,
    pub forwarded: DirectionStatistics
}

pub enum Scope {
    Single(FrameId),
    StandardRange(StandardId, StandardId),
    ExtendedRange(ExtendedId, ExtendedId),
    // Mask
}

impl Scope {
    pub fn contains(&self, other_id: FrameId) -> bool {
        match self {
            Scope::Single(id) => { *id == other_id }
            Scope::StandardRange(from, to) => {
                if let FrameId::Standard(sid) = other_id {
                    sid.id() >= from.id() && sid.id() <= to.id()
                } else {
                    false
                }
            }
            Scope::ExtendedRange(from, to) => {
                if let FrameId::Extended(eid) = other_id {
                    eid.id() >= from.id() && eid.id() <= to.id()
                } else {
                    false
                }
            }
        }
    }
}

pub struct RoutingEntry {
    scope: Scope,
    action: RoutingAction
}

pub type RxRoutingTable = heapless::Vec<RoutingEntry, config::RxRoutingTableSize>;

#[derive(PartialEq, Eq, Copy, Clone)]
pub struct ForwardEntry {
    pub to: Destination,
    pub frame: Frame
}
impl PartialOrd for ForwardEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for ForwardEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.frame.cmp(&other.frame)
    }
}

pub type ForwardHeap = heapless::BinaryHeap<ForwardEntry, config::ForwardHeapSize, heapless::binary_heap::Min>;

pub fn can0_rx_router(cx: crate::can0_rx_router::Context) {
    let receive_heap: &mut config::CanReceiveHeap = cx.resources.can0_receive_heap;
    let routing_table: &RxRoutingTable = cx.resources.can0_rx_routing_table;
    let statistics: &mut RxRoutingStatistics = cx.resources.can0_rx_routing_statistics;
    let local_processing_heap: &mut config::CanLocalProcessingHeap = cx.resources.can0_local_processing_heap;
    let mut channels = cx.resources.channels;

    while let Some(frame) = receive_heap.heap.pop() {
        for entry in routing_table {
            if entry.scope.contains(frame.id()) {
                match entry.action {
                    RoutingAction::Drop => {
                        statistics.dropped.frames_processed += 1;
                        statistics.dropped.bytes_processed += frame.len() as u32;
                    },
                    RoutingAction::ProcessLocally => {
                        match local_processing_heap.pool.new_frame(frame.id(), frame.data()) {
                            Ok(frame) => {
                                let len = frame.len() as u32;
                                match local_processing_heap.heap.push(frame) {
                                    Ok(_) => {
                                        statistics.processed_locally.frames_processed += 1;
                                        statistics.processed_locally.bytes_processed += len;
                                    },
                                    Err(_) => {
                                        statistics.processed_locally.frames_dropped += 1;
                                    }
                                }
                            },
                            Err(_) => {
                                statistics.processed_locally.frames_dropped += 1;
                            }
                        }
                    },
                    RoutingAction::Forward(destionation) => {
                        channels.lock(|channels| {
                            let forward_heap: &mut ForwardHeap = &mut channels.can0_forward_heap;
                            let forward_entry = ForwardEntry {
                                to: destionation,
                                frame
                            };
                            match forward_heap.push(forward_entry) {
                                Ok(_) => {
                                    statistics.forwarded.frames_processed += 1;
                                    statistics.forwarded.bytes_processed += frame.len() as u32;
                                },
                                Err(_) => {
                                    statistics.forwarded.frames_dropped += 1;
                                }
                            }
                        });
                    }
                }
            } else {
                statistics.dropped.frames_dropped += 1; // frames without action
            }
        }
    }
}

pub fn load_rx_routing_table(table: &mut RxRoutingTable) {
    // table.push(RoutingEntry{
    //     scope: Scope::Single(FrameId::new_extended(0x123).unwrap()),
    //     action: RoutingAction::Forward(Destination::Unicast(config::BL_UWB_ADDR))
    // });
    cfg_if! {
        if #[cfg(feature = "master")] {

        } else if #[cfg(feauture = "tr")] {

        } else if #[cfg(feature = "bl")] {

        } else if #[cfg(feature = "br")] {

        }
    }
}
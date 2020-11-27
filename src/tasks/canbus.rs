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
use core::fmt;
use no_std_compat::fmt::Formatter;
use crate::color;

#[derive(Default)]
pub struct DirectionStatistics {
    pub frames_processed: u32,
    pub frames_dropped: u32,
    pub bytes_processed: u32
}
impl fmt::Debug for DirectionStatistics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kib = self.bytes_processed / 1024;
        let b = self.bytes_processed % 1024;
        write!(f, "{} frames ({}KiB + {}) processed, {} dropped", self.frames_processed, kib, b, self.frames_dropped)
    }
}

#[derive(Default)]
pub struct LLStatistics {
    pub rx: DirectionStatistics,
    pub tx: DirectionStatistics
}

#[derive(Default)]
pub struct IrqStatistics {
    pub irqs: u32,
    pub bus_off: u32,
    pub lost: u32,
}

pub fn can0_irq0(mut cx: crate::can0_irq0::Context) {
    let can: &mut config::Can0 = cx.resources.can0;
    use crate::hal::can::ClassicalCan;
    let ll_statistics: &mut LLStatistics = cx.resources.can0_ll_statistics;
    let irq_statistics: &mut IrqStatistics = cx.resources.can0_irq_statistics;
    irq_statistics.irqs += 1;

    // rprintln!("can_irq0");
    // rprintln!("reason: {:?}", can.interrupt_reason());
    // rprintln!("{:?}", can.protocol_status());
    // rprintln!("rec:{} tec:{}", can.receive_error_counter(), can.transmit_error_counter());
    // rprintln!("rx_pin: {:?}", can.rx_pin_state());
    // rprintln!("queue: {}", cx.resources.can0_send_heap.heap.len());

    if can.protocol_status().is_bus_off() {
        // rprintln!("going out of bus off");
        irq_statistics.bus_off += 1;
        unsafe {
            can.ll(|can_regs| {
                can_regs.cccr.modify(|_, w| w.init().clear_bit()); // request normal mode
                can_regs.txbcr.write(|w| w.bits(0b111)); // cancel all outgoing frames
            });
        }
    } else {
        if can.free_slots() != 0 {
            cx.resources.channels.lock(|channels| {
                let send_heap: &mut config::CanSendHeap = &mut channels.can0_send_heap;
                if let Some(frame) = send_heap.heap.pop() {
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
            });
        }
    }

    unsafe {
        can.ll(|can_regs| {
            if can_regs.rxf0s.read().rf0l().bit_is_set() {
                irq_statistics.lost += 1;
            }
        });
    }

    let mut spawn_rx_router = false;
    let receive_heap: &mut config::CanReceiveHeap = cx.resources.can0_receive_heap;
    can.get_all(|id, data| {
        // rprintln!("rx: {:?}, {}", id, data.len());
        // for b in data {
        //     rprint!("{:02x} ", *b);
        // }
        // rprintln!("");
        match receive_heap.pool.new_frame(id, data) {
            Ok(frame) => {
                spawn_rx_router = true;
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
    if spawn_rx_router {
        let _ = cx.spawn.can0_rx_router();
    }

    unsafe { can.regs_mut().ir.write(|w| w.bits(0x00ff_ffff)) };
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Destination {
    Unicast(mac::ShortAddress),
    Multicast(mac::PanId),
    Broadcast
}

#[allow(dead_code)]
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

#[allow(dead_code)]
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
        //rprintln!(=>8, "pop:{:02x}", frame.data()[0]);
        let mut matches = false;
        for entry in routing_table {
            if entry.scope.contains(frame.id()) {
                matches = true;
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
                break;
            }
        }
        if !matches {
            statistics.dropped.frames_dropped += 1;
        }
    }
}

pub fn load_rx_routing_table(table: &mut RxRoutingTable) {
    // table.push(RoutingEntry{
    //     scope: Scope::Single(FrameId::new_extended(0x123).unwrap()),
    //     action: RoutingAction::Forward(Destination::Unicast(config::BL_UWB_ADDR))
    // });
    let mut failed = 0;
    cfg_if! {
        if #[cfg(feature = "master")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D22A).unwrap()), // Radar 1
                action: RoutingAction::Drop
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D32A).unwrap()), // Radar 2
                action: RoutingAction::Drop
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D42A).unwrap()), // Radar 3
                action: RoutingAction::Drop
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003840A).unwrap()), // Motor TL
                action: RoutingAction::Drop
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D80A).unwrap()), // Tacho TL
                action: RoutingAction::Drop
            }).is_ok() as u32;

            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003850A).unwrap()), // Motor TR
                action: RoutingAction::Forward(Destination::Unicast(config::TR_UWB_ADDR))
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003860A).unwrap()), // Motor BL
                action: RoutingAction::Forward(Destination::Unicast(config::BL_UWB_ADDR))
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003870A).unwrap()), // Motor BR
                action: RoutingAction::Forward(Destination::Unicast(config::BR_UWB_ADDR))
            }).is_ok() as u32;
        } else if #[cfg(feauture = "tr")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D90A).unwrap()), // Tacho TR
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
        } else if #[cfg(feature = "bl")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004DA0A).unwrap()), // Tacho BL
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
        } else if #[cfg(feature = "br")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D52A).unwrap()), // Radar 4
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D62A).unwrap()), // Radar 5
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D72A).unwrap()), // Radar 6
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004DB0A).unwrap()), // Tacho BR
                action: RoutingAction::Forward(Destination::Broadcast)
            }).is_ok() as u32;
        }
    }
    if failed > 0 {
        rprintln!(=>0, "\n\n{}load_rx_routing_table: table is not big enough{}", color::RED, color::DEFAULT);
    }
}
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
impl LLStatistics {
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[derive(Default)]
pub struct IrqStatistics {
    pub irqs: u32,
    pub bus_off: u32,
    pub lost: u32,
}
impl IrqStatistics {
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

pub fn can0_irq0(mut cx: crate::can0_irq0::Context) {
    let can: &mut config::Can0 = cx.resources.can0;
    use crate::hal::can::ClassicalCan;
    let ll_statistics: &mut LLStatistics = cx.resources.can0_ll_statistics;
    let irq_statistics: &mut IrqStatistics = cx.resources.can0_irq_statistics;
    irq_statistics.irqs += 1;

    // rprintln!("can_irq0");
    let interrupt_reason = can.interrupt_reason();
    //rprintln!("reason: {:?}", can.interrupt_reason());
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
        let channels = cx.resources.channels;
        let mut max_slots = 3;
        while can.free_slots() != 0 {
            // cx.resources.channels.lock(|channels| {
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
            max_slots -= 1;
            if max_slots == 0 {
                break;
            }
            // });
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

    unsafe { can.regs_mut().ir.write(|w| w.bits(interrupt_reason.0.bits())) };
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
impl RxRoutingStatistics {
    pub fn reset(&mut self) {
        *self = Self::default();
    }
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
    action: RoutingAction,
    comment: &'static str,
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

#[derive(PartialEq, Eq)]
pub struct CanIdCounter {
    pub count: u32,
    pub last_seen: rtic::cyccnt::Instant,
    pub cycle_time: rtic::cyccnt::Duration,
    pub comment: &'static str,
}
impl CanIdCounter {
    fn new() -> Self {
        CanIdCounter {
            count: 0,
            last_seen: rtic::cyccnt::Instant::now(),
            cycle_time: Default::default(),
            comment: "?"
        }
    }
}
pub type CanIdCounters = heapless::FnvIndexMap<vhrdcan::FrameId, CanIdCounter, heapless::consts::U32>;
pub struct CanAnalyzer {
    counters: CanIdCounters,
    overflow: bool
}
#[derive(PartialEq, Eq)]
pub enum CanAnalyzeStatus {
    Overflow,
    NewEntryCreated,
    Updated
}
impl CanAnalyzer {
    pub fn new() -> Self {
        CanAnalyzer {
            counters: CanIdCounters::new(),
            overflow: false
        }
    }

    pub fn analyze(&mut self, frame: &vhrdcan::Frame) -> CanAnalyzeStatus {
        let id = frame.id();
        let mut created_entry = false;
        if !self.counters.contains_key(&id) {
            match self.counters.insert(id, CanIdCounter::new()) {
                Ok(_) => {
                    created_entry = true;
                },
                Err(_) => {
                    self.overflow = true;
                    return CanAnalyzeStatus::Overflow;
                }
            }
        }
        match self.counters.get_mut(&frame.id()) {
            Some(counter) => {
                let now = rtic::cyccnt::Instant::now();
                counter.count += 1;
                counter.cycle_time = now.duration_since(counter.last_seen);
                counter.last_seen = now;
                if created_entry {
                    CanAnalyzeStatus::NewEntryCreated
                } else {
                    CanAnalyzeStatus::Updated
                }
            },
            None => {
                unreachable!();
            }
        }
    }

    pub fn add_comment(&mut self, id: &FrameId, comment: &'static str) {
        match self.counters.get_mut(id) {
            Some(counter) => {
                counter.comment = comment;
            },
            None => {}
        }
    }

    pub fn print_statistics(&self, clocks: &crate::hal::rcc::Clocks) {
        let mut i = 1;
        for (id, counter) in &self.counters {
            let cycle_time = cycles2us_alt!(clocks.sys_clk.0, counter.cycle_time.as_cycles());
            rprintln!(
                =>6,
                "{}.\t{:?}\t{}\t\t{}\t\t\tcycle: {}.{:03}ms",
                i, id, counter.comment, counter.count,
                cycle_time / 1000, cycle_time % 1000,
            );
            i += 1;
        }
        if self.overflow {
            rprintln!(=>6, "{}Analyzer overflow{}", color::YELLOW, color::DEFAULT);
        }
    }

    pub fn len(&self) -> usize {
        self.counters.len()
    }

    pub fn capacity(&self) -> usize {
        self.counters.capacity()
    }

    pub fn reset(&mut self) {
        self.counters.clear();
        self.overflow = false;
    }
}

pub fn can0_rx_router(cx: crate::can0_rx_router::Context) {
    let mut receive_heap = cx.resources.can0_receive_heap;
    let routing_table: &RxRoutingTable = cx.resources.can0_rx_routing_table;
    let statistics: &mut RxRoutingStatistics = cx.resources.can0_rx_routing_statistics;
    let local_processing_heap: &mut config::CanLocalProcessingHeap = cx.resources.can0_local_processing_heap;
    let mut channels = cx.resources.channels;
    let analyzer = cx.resources.can0_analyzer;

    loop {
        match receive_heap.lock(|rh: &mut config::CanReceiveHeap| rh.heap.pop()) {
            Some(frame) => {
                let analyze_status = analyzer.analyze(&frame);
                //rprintln!(=>8, "pop:{:02x}", frame.data()[0]);
                let mut matches = false;
                for entry in routing_table {
                    if entry.scope.contains(frame.id()) {
                        matches = true;
                        if analyze_status == CanAnalyzeStatus::NewEntryCreated {
                            analyzer.add_comment(&frame.id(), entry.comment);
                        }
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
            },
            None => {
                return;
            }
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
                action: RoutingAction::Drop,
                comment: "Radar1"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D32A).unwrap()), // Radar 2
                action: RoutingAction::Drop,
                comment: "Radar2"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D42A).unwrap()), // Radar 3
                action: RoutingAction::Drop,
                comment: "Radar3"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003840A).unwrap()), // Motor TL
                action: RoutingAction::Drop,
                comment: "Rpm_TL"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D80A).unwrap()), // Tacho TL
                action: RoutingAction::Drop,
                comment: "Tacho_TL"
            }).is_err() as u32;

            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003850A).unwrap()), // Motor TR
                action: RoutingAction::Forward(Destination::Unicast(config::TR_UWB_ADDR)),
                comment: "Rpm_TR"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003860A).unwrap()), // Motor BL
                action: RoutingAction::Forward(Destination::Unicast(config::BL_UWB_ADDR)),
                comment: "Rpm_BL"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1003870A).unwrap()), // Motor BR
                action: RoutingAction::Forward(Destination::Unicast(config::BR_UWB_ADDR)),
                comment: "Rpm_BR"
            }).is_err() as u32;
        } else if #[cfg(feauture = "tr")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D90A).unwrap()), // Tacho TR
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Tacho_TR"
            }).is_err() as u32;
        } else if #[cfg(feature = "bl")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004DA0A).unwrap()), // Tacho BL
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Tacho_BL"
            }).is_err() as u32;
        } else if #[cfg(feature = "br")] {
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D52A).unwrap()), // Radar 4
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Radar4"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D62A).unwrap()), // Radar 5
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Radar5"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004D72A).unwrap()), // Radar 6
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Radar6"
            }).is_err() as u32;
            failed += table.push(RoutingEntry{
                scope: Scope::Single(FrameId::new_extended(0x1004DB0A).unwrap()), // Tacho BR
                action: RoutingAction::Forward(Destination::Broadcast),
                comment: "Tacho_BR"
            }).is_err() as u32;
        }
    }
    if failed > 0 {
        rprintln!(=>0, "\n\n{}load_rx_routing_table: table is not big enough{}", color::RED, color::DEFAULT);
    }
}

#[derive(Eq, PartialEq)]
pub enum CanAnalyzerEvent {
    Print,
    Reset,
}
pub fn can_analyzer(mut cx: crate::can_analyzer::Context, e: CanAnalyzerEvent) {
    let reset = e == CanAnalyzerEvent::Reset;
    rprintln!(=>6, "");
    rprintln!(=>6, "\n\x1b[2J\x1b[0m---\n");
    cx.resources.can0_irq_statistics.lock(|irq_statistics: &mut crate::tasks::canbus::IrqStatistics| {
        if reset {
            irq_statistics.reset();
        }
        rprintln!(=>6, "IRQs: {}, BusOffs: {}, FramesLost: {}", irq_statistics.irqs, irq_statistics.bus_off, irq_statistics.lost);
    });
    cx.resources.can0_ll_statistics.lock(|can0_ll_statistics: &mut crate::tasks::canbus::LLStatistics| {
        if reset {
            can0_ll_statistics.reset();
        }
        rprintln!(=>6, "LL_RX: {:?}", can0_ll_statistics.rx);
        rprintln!(=>6, "LL_TX: {:?}", can0_ll_statistics.tx);
    });
    rprintln!(=>6, "");
    cx.resources.can0_rx_routing_statistics.lock(|can0_rx_routing_statistics: &mut crate::tasks::canbus::RxRoutingStatistics| {
        if reset {
            can0_rx_routing_statistics.reset();
        }
        rprintln!(=>6, "Route_DROP: {:?}", can0_rx_routing_statistics.dropped);
        rprintln!(=>6, "Route_PROCESS: {:?}", can0_rx_routing_statistics.processed_locally);
        rprintln!(=>6, "Route_FORWARD: {:?}", can0_rx_routing_statistics.forwarded);
    });
    let uwb_forwarding_counters = cx.resources.channels.lock(|channels| {
        if reset {
            channels.can2uwb = 0;
            channels.uwb2can_ok = 0;
            channels.uwb2can_drop = 0;
        }
        (channels.can2uwb, channels.uwb2can_ok, channels.uwb2can_drop)
    });
    rprintln!(=>6, "Can2Uwb: {}\tUwb2Can_OK: {}\tUwb2Can_DROP: {}", uwb_forwarding_counters.0, uwb_forwarding_counters.1, uwb_forwarding_counters.2);
    rprintln!(=>6, "");
    let clocks = cx.resources.clocks;

    cx.resources.can0_analyzer.lock(|can0_analyzer: &mut crate::tasks::canbus::CanAnalyzer| {
        rprintln!(=>6, "CAN RX Frames [{}]:", can0_analyzer.len());
        if reset {
            can0_analyzer.reset();
        }
        can0_analyzer.print_statistics(&clocks);
    });

    cx.resources.channels.lock(|channels: &mut crate::channels::Channels| {
        rprintln!(=>6, "\nCAN TX Frames [{}]:", channels.can0_forward_analyzer.len());
        if reset {
            channels.can0_forward_analyzer.reset();
        }
        channels.can0_forward_analyzer.print_statistics(clocks);
    });
    rprintln!(=>6, "\n=====\n");
}
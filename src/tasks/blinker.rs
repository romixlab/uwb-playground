use crate::config;

//use rtt_target::{rprint, rprintln};
use embedded_hal::digital::v2::ToggleableOutputPin;
use rtic::cyccnt::U32Ext;
use stm32g4xx_hal::pac::Interrupt;
use rtic::Mutex;
use rtt_target::rprintln;
use crate::tasks::canbus::{LLStatistics, RxRoutingStatistics};

#[derive(Default)]
pub struct CounterDeltas {
    can0_ll_statistics: LLStatistics,
    can0_rx_routing_statistics: RxRoutingStatistics,
    can2uwb: u32,
    uwb2can_ok: u32,
    uwb2can_drop: u32,
}

pub fn blinker(mut cx: crate::blinker::Context) {
    cx.resources.led_blinky.toggle().ok();

    rprintln!(=>1, "");
    rprintln!(=>1, "\n\x1b[2J\x1b[0m---\n");
    cx.resources.can0_irq_statistics.lock(|irq_statistics: &mut crate::tasks::canbus::IrqStatistics| {
        rprintln!(=>1, "IRQs: {}, BusOffs: {}", irq_statistics.irqs, irq_statistics.bus_off);
    });
    cx.resources.can0_ll_statistics.lock(|can0_ll_statistics: &mut crate::tasks::canbus::LLStatistics| {
        rprintln!(=>1, "LL_RX: {:?}", can0_ll_statistics.rx);
        rprintln!(=>1, "LL_TX: {:?}", can0_ll_statistics.tx);
    });
    cx.resources.can0_rx_routing_statistics.lock(|can0_rx_routing_statistics: &mut crate::tasks::canbus::RxRoutingStatistics| {
        rprintln!(=>1, "Route_DROP: {:?}", can0_rx_routing_statistics.dropped);
        rprintln!(=>1, "Route_PROCESS: {:?}", can0_rx_routing_statistics.processed_locally);
        rprintln!(=>1, "Route_FORWARD: {:?}", can0_rx_routing_statistics.forwarded);
    });
    let uwb_forwarding_counters = cx.resources.channels.lock(|channels| (channels.can2uwb, channels.uwb2can_ok, channels.uwb2can_drop));
    rprintln!(=>1, "Can2Uwb: {}\tUwb2Can_OK: {}\tUwb2Can_DROP: {}", uwb_forwarding_counters.0, uwb_forwarding_counters.1, uwb_forwarding_counters.2);

    cx.resources.channels.lock(|channels| {
        let h = &mut channels.can0_send_heap;
        h.heap.push(h.pool.new_frame(vhrdcan::FrameId::new_standard(0xac).unwrap(), &[0xaa, 0xbb, 0xcc]).unwrap());
    });
    rtic::pend(Interrupt::FDCAN1_INTR1_IT);

    cx.schedule.blinker(cx.scheduled + ms2cycles!(cx.resources.clocks, config::BLINK_PERIOD_MS)).ok(); // TODO: count errors
    // cx.schedule.blinker(cx.scheduled + us2cycles!(cx.resources.clocks, 300)).ok(); // TODO: count errors
}

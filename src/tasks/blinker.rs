use crate::config;

//use rtt_target::{rprint, rprintln};
use embedded_hal::digital::v2::ToggleableOutputPin;
use rtic::cyccnt::U32Ext;
use stm32g4xx_hal::pac::Interrupt;
use rtic::Mutex;

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

    cx.resources.can0_ll_statistics.lock(|can0_ll_statistics: &mut crate::tasks::canbus::LLStatistics| {

    });
    cx.resources.can0_rx_routing_statistics.lock(|can0_rx_routing_statistics: &mut crate::tasks::canbus::RxRoutingStatistics| {

    });
    let uwb_forwarding_counters = cx.resources.channels.lock(|channels| (channels.can2uwb, channels.uwb2can_ok, channels.uwb2can_drop));
    // cx.resources.can0_send_heap.lock(|h| {
    //     h.heap.push(h.pool.new_frame(vhrdcan::FrameId::new_standard(0xac).unwrap(), &[0xaa, 0xbb, 0xcc]).unwrap());
    // });
    // rtic::pend(Interrupt::FDCAN1_INTR1_IT);
    cx.schedule.blinker(cx.scheduled + ms2cycles!(cx.resources.clocks, config::BLINK_PERIOD_MS)).ok(); // TODO: count errors
    // cx.schedule.blinker(cx.scheduled + us2cycles!(cx.resources.clocks, 300)).ok(); // TODO: count errors
}

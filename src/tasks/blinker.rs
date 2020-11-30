use crate::config;

//use rtt_target::{rprint, rprintln};
use embedded_hal::digital::v2::ToggleableOutputPin;
use rtic::cyccnt::U32Ext;
use stm32g4xx_hal::pac::Interrupt;
use rtic::Mutex;
use rtt_target::rprintln;
use crate::tasks::canbus::{LLStatistics, RxRoutingStatistics};

#[allow(dead_code)]
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

    cx.resources.channels.lock(|channels| {
        let h = &mut channels.can0_send_heap;
        let _ = h.heap.push(h.pool.new_frame(vhrdcan::FrameId::new_standard(0xac).unwrap(), &[0xaa, 0xbb, 0xcc]).unwrap());
    });
    rtic::pend(config::CAN0_SEND_IRQ);

    cx.schedule.blinker(cx.scheduled + ms2cycles!(cx.resources.clocks, config::BLINK_PERIOD_MS)).ok(); // TODO: count errors
    let _ = cx.spawn.can_analyzer(crate::tasks::canbus::CanAnalyzerEvent::Print);
    // cx.schedule.blinker(cx.scheduled + us2cycles!(cx.resources.clocks, 300)).ok(); // TODO: count errors
}

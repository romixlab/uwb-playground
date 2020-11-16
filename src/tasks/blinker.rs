use crate::config;

//use rtt_target::{rprint, rprintln};
use embedded_hal::digital::v2::OutputPin;
use rtic::cyccnt::U32Ext;
use stm32g4xx_hal::pac::Interrupt;
use rtic::Mutex;

pub fn blinker(mut cx: crate::blinker::Context, led_state: &mut bool) {

    if *led_state {
        cx.resources.led_blinky.set_low().ok();
        *led_state = false;
    } else {
        cx.resources.led_blinky.set_high().ok();
        *led_state = true;
    }

    cx.resources.can0_send_heap.lock(|h| {
        h.heap.push(h.pool.new_frame(vhrdcan::FrameId::new_standard(123).unwrap(), &[1, 2, 3, 4]).unwrap());
    });
    //cx.spawn.lift_control(LiftControlCommand::AllUp);
    rtic::pend(Interrupt::FDCAN1_INTR0_IT);
    cx.schedule.blinker(cx.scheduled + ms2cycles!(cx.resources.clocks, config::BLINK_PERIOD_MS)).ok(); // TODO: count errors
}

use crate::config;

//use rtt_target::{rprint, rprintln};
use embedded_hal::digital::v2::OutputPin;
use rtic::cyccnt::U32Ext;

pub fn blinker(cx: crate::blinker::Context, led_state: &mut bool) {

    if *led_state {
        cx.resources.led_blinky.set_low().ok();
        *led_state = false;
    } else {
        cx.resources.led_blinky.set_high().ok();
        *led_state = true;
    }

    //cx.spawn.lift_control(LiftControlCommand::AllUp);
    cx.schedule.blinker(cx.scheduled + ms2cycles!(cx, config::BLINK_PERIOD_MS)).ok(); // TODO: count errors
}

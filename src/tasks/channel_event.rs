use rtt_target::rprintln;
use crate::motion::{
    Rpm,
    MotorControlEvent
};
use cfg_if::cfg_if;
use rtic::Mutex;

pub fn channel_event(mut cx: crate::channel_event::Context) {
    cfg_if! {
        if #[cfg(feature = "master")] {
            cx.resources.channels.lock(|channels| channels.grab_local_telemetry()); // maybe grab received telem from mc
            cx.spawn.ctrl_link_control(); // maybe send tacho/power array to ros
        } else if #[cfg(feature = "slave")] {
            match cx.resources.motion_channel_c.dequeue() {
                Some(rpm) => {
                    let r = cx.spawn.motor_control(MotorControlEvent::SetRpm(rpm));
                    cx.spawn.motor_control(MotorControlEvent::RequestTelemetry).ok();
                    rprintln!(=>14, "ch_ev: mc: {:?}", r);
                },
                None => {}
            }
        }
    }
}
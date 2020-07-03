use crate::config;
use crate::radio;
use crate::motion;
use crate::board::hal;

use rtic::Mutex;
use core::num::Wrapping;
use rtt_target::{rprint, rprintln};
use cortex_m::peripheral::DWT;
use rtic::cyccnt::U32Ext;
use hal::gpio::ExtiPin;
use embedded_hal::digital::v2::{OutputPin, InputPin, ToggleableOutputPin};

pub enum RadioChronoState {
    Idle,
    GTSInProgress,
}

pub fn radio_chrono(cx: crate::radio_chrono::Context, state: &mut RadioChronoState) {
    const GTS_END_MS: u32 = 30;
    //use radio::Command;
    //use radio::message::*;
    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                //let now = DWT::get_cycle_count();

                use crate::radio::message::{GTSEntry, GTSStart, GTSDownlinkData};
                use crate::radio::{Command};
                use RadioChronoState::*;
                *state = match state {
                    Idle => {
                        let wheels = cx.resources.mecanum_wheels;
                        let tr = GTSEntry::new(0, 6_000, GTSDownlinkData{ rpm: wheels.top_right.rpm.0 });
                        let bl = GTSEntry::new(16_000, 6_000, GTSDownlinkData{ rpm: wheels.bottom_left.rpm.0 });
                        let br = GTSEntry::new(22_000, 6_000, GTSDownlinkData{ rpm: wheels.bottom_right.rpm.0 });
                        let gts_start = GTSStart::new(tr, bl, br);
                        cx.resources.radio_commands_p.enqueue(Command::GTSStart(gts_start)).ok(); // TODO: Count errors
                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, GTS_END_MS)).ok(); // TODO: Count errors
                        //
                        RadioChronoState::GTSInProgress
                    },
                    GTSInProgress => {
                        cx.resources.radio_commands_p.enqueue(Command::GTSEnd).ok(); // TODO: Count errors
                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS - GTS_END_MS)).ok(); // TODO: Count errors
                        //
                        Idle
                    }
                }
            } else if #[cfg(all(feature = "slave", not(feature = "devnode")))] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok(); // TODO: count errors
            } else if #[cfg(feature = "devnode")] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok(); // TODO: count errors
            }
        }
    rtic::pend(config::DW1000_IRQ_EXTI);
}

pub fn radio_irq(cx: crate::radio_irq::Context, rx_buffer: &mut[u8], ) {
    // let now = DWT::get_cycle_count() as i32;
    // let dt = now.wrapping_sub(*LAST_IDLE_INSTANT);
    // *LAST_IDLE_INSTANT = now;
    // if dt > 72_000_000 * 2 {
    //     if *LAST_IDLE_COUNTER == *cx.resources.idle_counter {
    //         rprintln!("IRQ lockup detected!");
    //         cx.resources.radio_irq.disable_interrupt(cx.resources.exti);
    //     }
    //     *LAST_IDLE_COUNTER = *cx.resources.idle_counter;
    // }
    cx.resources.radio_trace.set_high().ok();
    cx.resources.radio_irq.clear_interrupt_pending_bit();
    //rprintln!("IRQ: {}us", cycles2us!(cx, dt));

    radio::state_machine::advance(
        cx.resources.radio_state,
        cx.resources.radio_commands_c,
        rx_buffer,
        &cx.spawn,
        cx.resources.radio_trace
    );
    for _ in 0..1000 {
        if cx.resources.radio_irq.is_high().unwrap() {
            cx.resources.radio_trace.toggle().ok();
            radio::state_machine::advance(
                cx.resources.radio_state,
                cx.resources.radio_commands_c,
                rx_buffer,
                &cx.spawn,
                cx.resources.radio_trace
            );
        } else {
            break;
        }
    }
    cx.resources.radio_trace.set_low().ok();
}

pub fn radio_event(mut cx: crate::radio_event::Context, e: radio::Event) {
    rprintln!("radio_event: {:?}", e);
    use radio::Event::*;
    match e {
        #[cfg(feature = "master")]
        GTSAnswerReceived(from, answer) => {
            let stat = cx.resources.stat;
            cx.resources.mecanum_wheels.lock(|wheels| {
                if from == config::TR_UWB_ADDR {
                    wheels.top_right.tacho = Tachometer(answer.data.tacho);
                    wheels.top_right.power_in = answer.data.power_in;
                    stat.tr_gts_answers += 1;
                } else if from == config::BL_UWB_ADDR {
                    wheels.bottom_left.tacho = Tachometer(answer.data.tacho);
                    wheels.bottom_left.power_in = answer.data.power_in;
                    stat.bl_gts_answers += 1;
                } else if from == config::BR_UWB_ADDR {
                    wheels.bottom_right.tacho = Tachometer(answer.data.tacho);
                    wheels.bottom_right.power_in = answer.data.power_in;
                    stat.br_gts_answers += 1;
                }
            });
        },
        #[cfg(feature = "devnode")]
        GTSAnswerReceived(from, answer) => {
            rprintln!("GTS answer rx from: {:?}, tacho: {}, pwr: {}", from, answer.data.tacho, answer.data.power_in);
        },
        #[cfg(all(feature = "slave", not(feature = "devnode")))]
        GTSStartReceived(tx_time, gts_end_dt, gts_entry) => {
            // Prepare telemetry for sending in a given slot
            let tacho = cx.resources.wheel.tacho;
            let power_in = cx.resources.wheel.power_in;
            let uplink_data = radio::message::GTSUplinkData { tacho: tacho.0, power_in };
            let gts_answer = radio::message::GTSAnswer { data: uplink_data };
            cx.resources.radio_commands_p.lock(|commands|
                commands.enqueue(radio::Command::GTSSendAnswer(tx_time, gts_answer))).ok(); // TODO: Count errors
            // Schedule an rpm sending after all GTS, so that each MC receive the command at the same time
            let rpm = motion::Rpm(gts_entry.sync_no_ack_data.rpm);
            cx.resources.wheel.rpm = rpm;
            cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 / 1000), radio::Event::GTSEnded).ok(); // TODO: count errors;
        },
        #[cfg(feature = "devnode")]
        GTSStartReceived(tx_time, gts_end_dt, gts_entry) => {
            rprintln!("GTS start received");
            cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 / 1000), radio::Event::GTSEnded);
        },
        GTSEnded => {
            cfg_if::cfg_if! {
                    if #[cfg(feature = "master")] {
                        let rpm = cx.resources.mecanum_wheels.lock(|wheels| wheels.top_left.rpm);
                        cx.spawn.ctrl_link_control().ok(); // TODO: count errors;
                    } else if  #[cfg(feature = "slave")] {
                        let rpm = cx.resources.wheel.rpm;
                        cx.spawn.motor_control(motion::MotorControlEvent::SetRpm(rpm)).ok(); // TODO: count errors
                    }
                }
            //cx.spawn.motor_control(MotorControlEvent::SetRpm(rpm)).ok(); // TODO: count errors
            cx.spawn.motor_control(motion::MotorControlEvent::RequestTelemetry).ok(); // TODO: count errors
        }
    }
}

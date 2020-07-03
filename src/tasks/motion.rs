use crate::config;
use crate::motion;
use crate::board::hal;
use crate::crc_framer;

use rtt_target::rprintln;
use rtic::cyccnt::U32Ext;
use rtic::Mutex;

pub fn motor_control(
    mut cx: crate::motor_control::Context,
    e: motion::MotorControlEvent,
    last_command_instant: &mut Option<rtic::cyccnt::Instant>,
    stopped: &mut bool
) {
    //rprintln!(=> 9, "mc_event: {:?}\n", e);
    use motion::MotorControlEvent;
    use motion::MotorControlEvent::*;
    // Schedule timing check when first move event is received
    if e.is_move_event() {
        if last_command_instant.is_none() {
            cx.schedule.motor_control(
                cx.scheduled + ms2cycles!(cx, config::motor_control::TIMING_CHECK_INTERVAL_MS),
                motion::MotorControlEvent::TimingCheck
            ).ok(); // TODO: count errors, totally fail at this probably
        }
        *last_command_instant = Some(rtic::cyccnt::Instant::now());
    }
    match e {
        #[cfg(feature = "master")]
        SetRpmArray(rpms) => {
            rprintln!(=> 9, "SetRpmArray: t:{:?} {:?}\n", rtic::cyccnt::Instant::now(), rpms);
            cx.resources.mecanum_wheels.lock(|wheels| {
                wheels.top_left.rpm = rpms.top_left;
                wheels.top_right.rpm = rpms.top_right;
                wheels.bottom_left.rpm = rpms.bottom_left;
                wheels.bottom_right.rpm = rpms.bottom_right;
            });
            cx.spawn.motor_control( // TODO: schedule at the of GTS to send in sync with slaves
                                    MotorControlEvent::SetRpm(rpms.top_left)
            ).ok(); // TODO: count errors
        },
        #[cfg(feature = "master")]
        MovePreserveHeading(_xyt) => {

        },
        #[cfg(feature = "master")]
        MoveIgnoreHeading(_xyt) => {

        },
        SetRpm(rpm) => {
            rprintln!(=> 9, "SetRpm t:{:?} {}\n", rtic::cyccnt::Instant::now(), rpm.0);
            if rpm.0 != 0 {
                let mut setrpm_frame = [0u8; 5];
                setrpm_frame[0] = config::VESC_SETRPM_FRAME_ID;
                setrpm_frame[1..=4].copy_from_slice(&rpm.0.to_be_bytes());
                crc_framer::CrcFramerSer::commit_frame(&setrpm_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok(); // TODO: count errors
                *stopped = false;
            } else if rpm.0 == 0 && !*stopped {
                let mut setcurrent_frame = [0u8; 5];
                setcurrent_frame[0] = config::VESC_SETCURRENT_FRAME_ID;
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok(); // TODO: count errors
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok();
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok();
                *stopped = true; // do not send 0 all the time, or tacho will count for some reason
                rprintln!(=> 9, "SetI = 0\n");
            }
        },
        RequestTelemetry => {
            let mut request = [0u8; 5];
            request[0] = config::VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID;
            request[1..=4].copy_from_slice(&config::VESC_REQUESTED_VALUES.to_be_bytes());
            crc_framer::CrcFramerSer::commit_frame(&request, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok(); // TODO: count errors
        },
        TimingCheck => {
            let dt = last_command_instant.expect("motor_control::TimingCheck fail").elapsed();

            let stop_timeout_cycles: u32 = ms2cycles_raw!(cx, config::motor_control::STOP_TIMEOUT_MS);
            rprintln!(=> 9, "TimingCheck dt:{} thresh:{}\n", dt.as_cycles(), stop_timeout_cycles,);

            if dt.as_cycles() >= stop_timeout_cycles {
                rprintln!(=> 9, "ALL STOP {}\n", cycles2ms!(cx, dt.as_cycles()));
                cfg_if::cfg_if! {
                        if #[cfg(feature = "master")] {
                            cx.resources.mecanum_wheels.lock(|wheels| wheels.all_stop() );
                        } else if #[cfg(feature = "slave")] {
                            cx.resources.wheel.lock(|wheel| wheel.stop() );
                        }
                    }
                let mut setcurrent_frame = [0u8; 5];
                setcurrent_frame[0] = config::VESC_SETCURRENT_FRAME_ID;
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok(); // TODO: count errors
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok();
                crc_framer::CrcFramerSer::commit_frame(&setcurrent_frame, cx.resources.vesc_bbbuffer_p, config::VESC_IRQ_EXTI).ok();
                *stopped = true; // do not send 0 all the time, or tacho will count for some reason
            }

            cx.schedule.motor_control(
                cx.scheduled + ms2cycles!(cx, config::motor_control::TIMING_CHECK_INTERVAL_MS),
                motion::MotorControlEvent::TimingCheck
            ).ok(); // TODO: count errors, totally fail at this probably
        },
        #[cfg(feature = "master")]
        ResetTacho => {
            cfg_if::cfg_if! {
                    if #[cfg(feature = "master")] {
                        cx.resources.mecanum_wheels.lock(|wheels| {
                            wheels.top_left.tacho_shift = wheels.top_left.tacho.0;
                            wheels.top_right.tacho_shift = wheels.top_right.tacho.0;
                            wheels.bottom_left.tacho_shift = wheels.bottom_left.tacho.0;
                            wheels.bottom_right.tacho_shift = wheels.bottom_right.tacho.0;
                        });
                    }
                }
        }
    }
}

pub fn vesc_serial_irq(
    cx: crate::vesc_serial_irq::Context,
    sending: &mut Option<bbqueue::GrantR<'static, config::VescBBBufferSize>>,
    sending_idx: &mut usize
) {
    use core::convert::TryInto;
    use embedded_hal::serial::{Read, Write};

    let serial = cx.resources.vesc_serial;
    if serial.is_rxne() {
        match serial.read() {
            Ok(byte) => {
                let framer = cx.resources.vesc_framer;
                //let bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
                cfg_if::cfg_if! {
                        if #[cfg(feature = "master")] {
                            let mut wheels = cx.resources.mecanum_wheels;
                        } else if  #[cfg(feature = "slave")] {
                            let mut wheel = cx.resources.wheel;
                        }
                    }
                //rprintln!(=> 5, "{} {:02x}\n", byte, byte);
                framer.eat_byte(byte, |frame| {
                    //rprintln!(=> 5, "frame vesc: {} {}\n", frame[0], frame.len());
                    if frame[0] == config::VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID && frame.len() == 15 {
                        let current_in = i32::from_be_bytes(frame[5..=8].try_into().unwrap());
                        let current_in = (current_in as f32) / 100.0f32;
                        let voltage_in = i16::from_be_bytes(frame[9..=10].try_into().unwrap());
                        let voltage_in = voltage_in as f32;
                        let power_in = current_in * voltage_in;
                        let tacho = i32::from_be_bytes(frame[11..=14].try_into().unwrap());
                        let tacho = motion::Tachometer(tacho);
                        //rprintln!(=> 5, "i_in:{} v_in:{} tacho:{}", current_in, voltage_in, tacho.0);

                        cfg_if::cfg_if! {
                                if #[cfg(feature = "master")] {
                                    wheels.lock(|wheels| {
                                        wheels.top_left.tacho = tacho;
                                        wheels.top_left.power_in = power_in;
                                    });
                                } else if  #[cfg(feature = "slave")] {
                                    wheel.lock(|wheel| {
                                        wheel.tacho = tacho;
                                        wheel.power_in = power_in;
                                    });
                                }
                            }
                    }
                });
            },
            _ => {}
        }
    }

    use hal::serial::Event;
    //let _:() = cx.resources.vesc_bbbuffer_c.read().unwrap();
    *sending = if sending.is_some() {
        let rgr = sending.take().unwrap();
        let already_sent = *sending_idx;
        if already_sent == rgr.len() {
            rgr.release(already_sent); // Previous block is finished
            serial.unlisten(Event::Txe);
            None // Try send more!
        } else {
            let next = rgr[already_sent];
            match serial.write(next) {
                Ok(_) => {
                    *sending_idx += 1;
                    Some(rgr)
                },
                _ => {
                    Some(rgr)
                }
            }
        }
    } else {
        match cx.resources.vesc_bbbuffer_c.read() {
            Ok(rgr) => {
                let next = rgr[0];
                serial.listen(Event::Txe);
                match serial.write(next) {
                    Ok(_) => {
                        *sending_idx = 1;
                        Some(rgr)
                    },
                    _ => {
                        *sending_idx = 0;
                        Some(rgr)
                    }
                }
            },
            Err(_) => {
                None
            }
        }
    }
}

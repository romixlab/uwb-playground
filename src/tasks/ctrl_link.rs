use crate::config;
use crate::board::hal;

use rtt_target::{rprint, rprintln};

pub fn ctrl_link_control(mut cx: crate::ctrl_link_control::Context) {
    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let (tacho_tl, tacho_tr, tacho_bl, tacho_br) = cx.resources.mecanum_wheels.lock(|wheels| {
                    (
                        wheels.top_left.tacho.0 - wheels.top_left.tacho_shift,
                        wheels.top_right.tacho.0 - wheels.top_right.tacho_shift,
                        wheels.bottom_left.tacho.0 - wheels.bottom_left.tacho_shift,
                        wheels.bottom_right.tacho.0 - wheels.bottom_right.tacho_shift
                    )
                });
                let mut tacho_arr_frame = [0u8; 17];
                tacho_arr_frame[0] = VESC_TACHO_ARRAY_FRAME_ID;
                tacho_arr_frame[1..=4].copy_from_slice(&tacho_tl.to_be_bytes());
                tacho_arr_frame[5..=8].copy_from_slice(&tacho_tr.to_be_bytes());
                tacho_arr_frame[9..=12].copy_from_slice(&tacho_bl.to_be_bytes());
                tacho_arr_frame[13..=16].copy_from_slice(&tacho_br.to_be_bytes());
                cx.resources.ctrl_bbbuffer_p.lock(|bb| {
                    crc_framer::CrcFramerSer::commit_frame(&tacho_arr_frame, bb, CTRL_IRQ_EXTI).ok(); // TODO: count errors
                });
            }
        }
}

pub fn ctrl_serial_irq(
    cx: crate::ctrl_serial_irq::Context,
    sending: &mut Option<bbqueue::GrantR<'static, config::CtrlBBBufferSize>>,
    sending_idx: &mut usize
) {
    let serial = cx.resources.ctrl_serial;
    use core::convert::TryInto;

    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                match serial.read() {
                    Ok(byte) => {
                        let framer = cx.resources.ctrl_framer;
                        //let bbbuffer_p = cx.resources.vesc_bbbuffer_p;
                        let spawner = cx.spawn;
                        framer.eat_byte(byte, |frame| {
                            rprintln!(=> 5, "frame ctrl: {} {}\n", frame[0], frame.len());
                            if frame[0] == config::VESC_RPM_ARRAY_FRAME_ID {
                                let tl_rpm = Rpm( i32::from_be_bytes(frame[1..=4].try_into().unwrap()) );
                                let tr_rpm = Rpm( i32::from_be_bytes(frame[5..=8].try_into().unwrap()) );
                                let bl_rpm = Rpm( i32::from_be_bytes(frame[9..=12].try_into().unwrap()) );
                                let br_rpm = Rpm( i32::from_be_bytes(frame[13..=16].try_into().unwrap()) );
                                let rpm_array = RpmArray {
                                    top_left: tl_rpm,
                                    top_right: tr_rpm,
                                    bottom_left: bl_rpm,
                                    bottom_right: br_rpm
                                };
                                spawner.motor_control(MotorControlEvent::SetRpmArray(rpm_array)).ok(); // TODO: count errors;
                                //rprintln!("{} {} {} {}", tl_rpm, tr_rpm, bl_rpm, br_rpm);
                            } else if frame[0] == config::VESC_LIFT_FRAME_ID && frame.len() == 2 {
                                rprintln!(=>5, "lift cmd: {}", frame[1] as char);
                                use LiftControlCommand::*;
                                match frame[1] as char {
                                    'q' => { spawner.lift_control(LeftDown).ok(); },
                                    'a' => { spawner.lift_control(LeftUp).ok(); },
                                    'w' => { spawner.lift_control(AllDown).ok(); },
                                    's' => { spawner.lift_control(AllUp).ok(); },
                                    'e' => { spawner.lift_control(RightDown).ok(); },
                                    'd' => { spawner.lift_control(RightUp).ok(); },
                                    _ => {}
                                };
                            } else if frame[0] == config::VESC_RESET_ALL {
                                rprintln!(=>5, "\n\nRESET TACHO\n\n");
                                spawner.motor_control(MotorControlEvent::ResetTacho).ok(); // TODO: count errors;
                            }
                            //crc_framer::CrcFramerSer::commit_frame(frame, bbbuffer_p);
                            //rtic::pend(VESC_IRQ_EXTI);
                        });
                    },
                    _ => {}
                }
            } else if #[cfg(feature = "br")] {

            }
        }


    use hal::serial::Event;
    use embedded_hal::serial::Write;
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
        match cx.resources.ctrl_bbbuffer_c.read() {
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

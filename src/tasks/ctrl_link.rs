use crate::config;
use crate::board::hal;

//use rtt_target::{rprint, rprintln};

#[allow(unused_variables)]
pub fn ctrl_link_control(mut cx: crate::ctrl_link_control::Context) {
    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            use crate::crc_framer;
            use rtic::Mutex;

            let (tacho_tl, tacho_tr, tacho_bl, tacho_br) = cx.resources.mecanum_wheels.lock(|wheels| {
                (
                    wheels.top_left.tacho.0 - wheels.top_left.tacho_shift,
                    wheels.top_right.tacho.0 - wheels.top_right.tacho_shift,
                    wheels.bottom_left.tacho.0 - wheels.bottom_left.tacho_shift,
                    wheels.bottom_right.tacho.0 - wheels.bottom_right.tacho_shift
                )
            });
            let mut tacho_arr_frame = [0u8; 17];
            tacho_arr_frame[0] = config::VESC_TACHO_ARRAY_FRAME_ID;
            tacho_arr_frame[1..=4].copy_from_slice(&tacho_tl.to_be_bytes());
            tacho_arr_frame[5..=8].copy_from_slice(&tacho_tr.to_be_bytes());
            tacho_arr_frame[9..=12].copy_from_slice(&tacho_bl.to_be_bytes());
            tacho_arr_frame[13..=16].copy_from_slice(&tacho_br.to_be_bytes());
            cx.resources.ctrl_bbbuffer_p.lock(|bb| {
                crc_framer::CrcFramerSer::commit_frame(&tacho_arr_frame, bb, config::CTRL_IRQ_EXTI).ok(); // TODO: count errors
            });
        } else if #[cfg(feature = "br")] {
            use rtic::Mutex;
            use rtt_target::{rprint, rprintln};

            while cx.resources.lidar_queue_c.ready() {
                match cx.resources.lidar_queue_c.dequeue() {
                    Some(frame) => {
                        let scan = frame.0;
                        let angle: u16 = ((scan[3] & 0b0111_1111) as u16) << 8 | scan[2] as u16;
                        let angle_dec = angle / 64;
                        let angle_frac = angle % 64;
                        rprintln!(=> 5, "{}.{}\n", angle_dec, angle_frac);
                        for i in 0..84 {
                            rprint!(=>5, "{:02x} ", scan[i]);
                        }
                        rprintln!(=>5, "]\n");
                    },
                    None => { }
                }
            }

            rprintln!(=>5, ".");
            let mut ctrl_bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
            cx.resources.lidar.lock(|lidar|
                lidar.periodic_check(|tx_bytes| {
                    rprint!(=>5, "ctrl:tx:[");
                    for b in tx_bytes {
                        rprint!(=>5, "{:02x} ", b);
                    }
                    rprintln!(=>5, "]");
                    ctrl_bbbuffer_p.lock(|bb| {
                        match bb.grant_exact(tx_bytes.len()) {

                            Ok(mut wgr) => {
                                wgr.copy_from_slice(tx_bytes);
                                wgr.commit(tx_bytes.len());
                                rtic::pend(config::CTRL_IRQ_EXTI);
                            },
                            Err(_) => {
                                rprintln!(=>5, "  failed");
                            }
                        }
                    });
                })
            );
            use rtic::cyccnt::U32Ext;
            cx.schedule.ctrl_link_control(cx.scheduled + ms2cycles!(cx, 50)).ok();
        }
    }
}

pub fn ctrl_serial_irq(
    cx: crate::ctrl_serial_irq::Context,
    sending: &mut Option<bbqueue::GrantR<'static, config::CtrlBBBufferSize>>,
    sending_idx: &mut usize
) {
    let serial = cx.resources.ctrl_serial;
    //rprintln!(=> 5, "irq");

    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            use core::convert::TryInto;
            use rtt_target::rprintln;
            use crate::motion::{MotorControlEvent, Rpm, RpmArray};
            use embedded_hal::serial::Read;

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
                            use crate::tasks::lift::LiftControlCommand::*;
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
            use embedded_hal::serial::Read;
            use rtt_target::{rprint, rprintln};
            use crate::rplidar;
            match serial.read() {
                Ok(byte) => {
                    let lidar = cx.resources.lidar;
                    //let spawner = cx.spawn;
                    let bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
                    let lidar_queue_p = cx.resources.lidar_queue_p;
                    //rprintln!(=>5, "r:{:02x}\n", byte);
                    lidar.eat_byte(byte,
                        |scan| {
                            if scan.len() == rplidar::FRAME_SIZE {
                                let mut frame = [0u8; 84];
                                frame.copy_from_slice(&scan[..rplidar::FRAME_SIZE]);
                                let r = lidar_queue_p.enqueue(rplidar::Frame(frame));
                                if r.is_err() {
                                    rprintln!(=> 5, "drop");
                                }
                            }
                        },
                        |tx_bytes| {
                            rprint!(=>5, "ctrl_irq:tx:[");
                            for b in tx_bytes {
                                rprint!(=>5, "{:02x} ", b);
                            }
                            rprintln!(=>5, "]\n");
                            match bbbuffer_p.grant_exact(tx_bytes.len()) {
                                Ok(mut wgr) => {
                                    wgr.copy_from_slice(tx_bytes);
                                    wgr.commit(tx_bytes.len());
                                    rtic::pend(config::CTRL_IRQ_EXTI);
                                },
                                Err(_) => {
                                    rprintln!(=>5, "  failed");
                                }
                            }
                        }
                    );
                },
                Err(e) => {

                }
            }
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

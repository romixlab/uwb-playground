use crate::config;
use crate::board::hal;
use rtic::Mutex;
use rtt_target::rprintln;
use crate::motion::{
    TelemetryArray,
    TachoArray,
    PowerArray,
};

#[allow(unused_variables)]
pub fn ctrl_link_control(mut cx: crate::ctrl_link_control::Context) {
    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            use crate::crc_framer;
            use rtic::Mutex;

            loop {
                let mut frame = [0u8; 17];
                match cx.resources.motion_telemetry_c.dequeue() {
                    Some(telem_array) => {
                        match telem_array {
                            TelemetryArray::Tachometer(tacho_array) => {
                                frame[0] = config::VESC_TACHO_ARRAY_FRAME_ID;
                                frame[1..=4].copy_from_slice(&tacho_array.top_left.0.to_be_bytes());
                                frame[5..=8].copy_from_slice(&tacho_array.top_right.0.to_be_bytes());
                                frame[9..=12].copy_from_slice(&tacho_array.bottom_left.0.to_be_bytes());
                                frame[13..=16].copy_from_slice(&tacho_array.bottom_right.0.to_be_bytes());
                            },
                            TelemetryArray::Power(power_array) => {
                                frame[0] = config::VESC_POWER_ARRAY_FRAME_ID;
                                frame[1..=4].copy_from_slice(&power_array.top_left.0.to_bits().to_be_bytes());
                                frame[5..=8].copy_from_slice(&power_array.top_right.0.to_bits().to_be_bytes());
                                frame[9..=12].copy_from_slice(&power_array.bottom_left.0.to_bits().to_be_bytes());
                                frame[13..=16].copy_from_slice(&power_array.bottom_right.0.to_bits().to_be_bytes());
                            }
                        }
                        cx.resources.ctrl_bbbuffer_p.lock(|bb| {
                            crc_framer::CrcFramerSer::commit_frame(&frame, bb, config::CTRL_IRQ_EXTI).ok(); // TODO: count errors
                        });
                        rprintln!(=>5, "telem array consumed");
                    },
                    None => {
                        break;
                    }
                }
            }
            while cx.resources.lidar_queue_c.ready() {
                match cx.resources.lidar_queue_c.dequeue() {
                    Some(frame) => {
                        let mut vesc_frame = [0u8; crate::rplidar::FRAME_SIZE + 1];
                        vesc_frame[0] = config::VESC_LIDAR_FRAME_ID;
                        vesc_frame[1..].copy_from_slice(&frame.0);
                        cx.resources.ctrl_bbbuffer_p.lock(|bb| {
                            crc_framer::CrcFramerSer::commit_frame(&vesc_frame, bb, config::CTRL_IRQ_EXTI).ok();
                        });
                    },
                    None => { }
                }
            }
        } else if #[cfg(feature = "br")] {
            use rtic::Mutex;
            use rtt_target::{rprint, rprintln};

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
            cx.schedule.ctrl_link_control(cx.scheduled + ms2cycles!(cx.resources.clocks, 50)).ok();
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
                    let motion_channel_p = cx.resources.motion_channel_p;
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
                            spawner.motor_control(MotorControlEvent::SetRpm(tl_rpm)).ok(); // TODO: count errors;
                            spawner.motor_control(MotorControlEvent::RequestTelemetry).ok();
                            let r = motion_channel_p.enqueue(rpm_array);
                            rprintln!(=> 5, "enqueue RA:{:?}", r);
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
                            //spawner.motor_control(MotorControlEvent::ResetTacho).ok(); // TODO: count errors;
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
                    //let lidar_queue_p = cx.resources.lidar_queue_p;
                    //rprintln!(=>5, "r:{:02x}\n", byte);
                    lidar.eat_byte(byte,
                        // |scan| {
                        //     if scan.len() == rplidar::FRAME_SIZE {
                        //         // let mut frame: [0u8; rplidar::FRAME_SIZE] = unsafe { core::mem::MaybeUninit() };
                        //         // frame.copy_from_slice(&scan[..rplidar::FRAME_SIZE]);
                        //         // let r = lidar_queue_p.enqueue(rplidar::Frame(frame));
                        //         // if r.is_err() {
                        //         //     rprintln!(=> 5, "drop");
                        //         // }
                        //     }
                        // },
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

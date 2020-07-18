use crate::config;
use crate::board::hal;
use rtic::Mutex;
use rtt_target::rprintln;
use crate::motion::{
    TelemetryArray,
    TachoArray,
    PowerArray,
};
use crate::color;

#[derive(Debug)]
pub enum Usart1WorkerEvent {
    Rx,

}

pub fn usart1_worker(mut cx: crate::usart1_worker::Context, e: Usart1WorkerEvent) {

}

#[derive(Debug)]
pub enum Usart2WorkerEvent {
    Rx,

}

#[cfg(feature = "master")]
pub fn usart2_worker(mut cx: crate::usart2_worker::Context, e: Usart2WorkerEvent) {
    rprintln!(=>5, "usart2_worker: {:?}\n", e);
    match e {
        Usart2WorkerEvent::Rx => {
            cx.resources.usart2_decoder.consume_all(&mut |frame: &[u8]| {
                rprintln!(=>5, "frame: {}", frame.len());
            });
        },
    }
}

#[cfg(feature = "br")]
pub fn usart2_worker(mut cx: crate::usart2_worker::Context, e: Usart2WorkerEvent) {
    rprintln!(=>5, "usart2_worker: {:?}\n", e);
}

// #[allow(unused_variables)]
// pub fn ctrl_link_control(mut cx: crate::ctrl_link_control::Context) {
//     cfg_if::cfg_if! {
//         if #[cfg(feature = "master")] {
//             use crate::codec;
//             use rtic::Mutex;
//
//             loop {
//                 let mut frame = [0u8; 17];
//                 match cx.resources.motion_telemetry_c.dequeue() {
//                     Some(telem_array) => {
//                         match telem_array {
//                             TelemetryArray::Tachometer(tacho_array) => {
//                                 frame[0] = config::VESC_TACHO_ARRAY_FRAME_ID;
//                                 frame[1..=4].copy_from_slice(&tacho_array.top_left.0.to_be_bytes());
//                                 frame[5..=8].copy_from_slice(&tacho_array.top_right.0.to_be_bytes());
//                                 frame[9..=12].copy_from_slice(&tacho_array.bottom_left.0.to_be_bytes());
//                                 frame[13..=16].copy_from_slice(&tacho_array.bottom_right.0.to_be_bytes());
//                             },
//                             TelemetryArray::Power(power_array) => {
//                                 frame[0] = config::VESC_POWER_ARRAY_FRAME_ID;
//                                 frame[1..=4].copy_from_slice(&power_array.top_left.0.to_bits().to_be_bytes());
//                                 frame[5..=8].copy_from_slice(&power_array.top_right.0.to_bits().to_be_bytes());
//                                 frame[9..=12].copy_from_slice(&power_array.bottom_left.0.to_bits().to_be_bytes());
//                                 frame[13..=16].copy_from_slice(&power_array.bottom_right.0.to_bits().to_be_bytes());
//                             }
//                         }
//                         cx.resources.ctrl_bbbuffer_p.lock(|bb| {
//                             codec::CrcFramerSer::commit_frame(&frame, bb, config::CTRL_TX_DMA).ok(); // TODO: count errors
//                         });
//                         rprintln!(=>5, "telem array consumed");
//                     },
//                     None => {
//                         break;
//                     }
//                 }
//             }
//             loop {
//                 let rgr = cx.resources.lidar_frame_c.read();
//                 match rgr {
//                     Some(rgr) => {
//                         let mut vesc_frame = [0u8; crate::rplidar::FRAME_SIZE + 1];
//                         vesc_frame[0] = config::VESC_LIDAR_FRAME_ID;
//                         vesc_frame[1..].copy_from_slice(&rgr);
//                         let r = cx.resources.ctrl_bbbuffer_p.lock(|bb| {
//                             codec::CrcFramerSer::commit_frame(&vesc_frame, bb, config::CTRL_TX_DMA)
//                         });
//                         rgr.release();
//                         if r.is_ok() {
//                             rprintln!(=>15, "{}l.sent_to_ros{}", color::GREEN, color::DEFAULT);
//                         } else {
//                             rprintln!(=>15, "{}l.DROP_in_serial{}", color::YELLOW, color::DEFAULT);
//                             break;
//                         }
//                     },
//                     None => { break; }
//                 }
//             }
//         } else if #[cfg(feature = "br")] {
//             use rtic::Mutex;
//             use rtt_target::{rprint, rprintln};
//
//             //rprintln!(=>5, ".");
//             let mut ctrl_bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
//             cx.resources.lidar.lock(|lidar|
//                 lidar.periodic_check(|tx_bytes| {
//                     rprint!(=>5, "ctrl:tx:[");
//                     for b in tx_bytes {
//                         rprint!(=>5, "{:02x} ", b);
//                     }
//                     rprintln!(=>5, "]");
//                     ctrl_bbbuffer_p.lock(|bb| {
//                         match bb.grant_exact(tx_bytes.len()) {
//
//                             Ok(mut wgr) => {
//                                 wgr.copy_from_slice(tx_bytes);
//                                 wgr.commit(tx_bytes.len());
//                                 rtic::pend(config::CTRL_TX_DMA);
//                             },
//                             Err(_) => {
//                                 rprintln!(=>5, "  failed");
//                             }
//                         }
//                     });
//                 })
//             );
//             use rtic::cyccnt::U32Ext;
//             cx.schedule.ctrl_link_control(cx.scheduled + ms2cycles!(cx.resources.clocks, 50)).ok();
//         }
//     }
// }
//
// pub fn ctrl_serial_irq(
//     cx: crate::ctrl_serial_irq::Context,
// ) {
//     let serial = cx.resources.ctrl_serial;
//     let bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
//
//     cfg_if::cfg_if! {
//         if #[cfg(feature = "master")] {
//             use core::convert::TryInto;
//             use rtt_target::rprintln;
//             use crate::motion::{MotorControlEvent, Rpm, RpmArray};
//             use embedded_hal::serial::Read;
//
//             match serial.read() {
//                 Ok(byte) => {
//                     let framer = cx.resources.ctrl_framer;
//                     //let bbbuffer_p = cx.resources.vesc_bbbuffer_p;
//                     let motion_channel_p = cx.resources.motion_channel_p;
//                     let spawner = cx.spawn;
//                     rprintln!(=> 5, "{:02x}", byte);
//                     framer.eat_byte(byte, |frame| {
//                         rprintln!(=> 5, "frame ctrl: {} {}\n", frame[0], frame.len());
//                         if frame[0] == config::VESC_RPM_ARRAY_FRAME_ID {
//                             let tl_rpm = Rpm( i32::from_be_bytes(frame[1..=4].try_into().unwrap()) );
//                             let tr_rpm = Rpm( i32::from_be_bytes(frame[5..=8].try_into().unwrap()) );
//                             let bl_rpm = Rpm( i32::from_be_bytes(frame[9..=12].try_into().unwrap()) );
//                             let br_rpm = Rpm( i32::from_be_bytes(frame[13..=16].try_into().unwrap()) );
//                             let rpm_array = RpmArray {
//                                 top_left: tl_rpm,
//                                 top_right: tr_rpm,
//                                 bottom_left: bl_rpm,
//                                 bottom_right: br_rpm
//                             };
//                             spawner.motor_control(MotorControlEvent::SetRpm(tl_rpm)).ok(); // TODO: count errors;
//                             spawner.motor_control(MotorControlEvent::RequestTelemetry).ok();
//                             let r = motion_channel_p.enqueue(rpm_array);
//                             rprintln!(=> 5, "enqueue RA:{} {:?} {:?}", r.is_ok(), tl_rpm, br_rpm);
//                             //rprintln!("{} {} {} {}", tl_rpm, tr_rpm, bl_rpm, br_rpm);
//                         } else if frame[0] == config::VESC_LIFT_FRAME_ID && frame.len() == 2 {
//                             rprintln!(=>5, "lift cmd: {}", frame[1] as char);
//                             use crate::tasks::lift::LiftControlCommand::*;
//                             match frame[1] as char {
//                                 'q' => { spawner.lift_control(LeftDown).ok(); },
//                                 'a' => { spawner.lift_control(LeftUp).ok(); },
//                                 'w' => { spawner.lift_control(AllDown).ok(); },
//                                 's' => { spawner.lift_control(AllUp).ok(); },
//                                 'e' => { spawner.lift_control(RightDown).ok(); },
//                                 'd' => { spawner.lift_control(RightUp).ok(); },
//                                 _ => {}
//                             };
//                         } else if frame[0] == config::VESC_RESET_ALL {
//                             rprintln!(=>5, "\n\nRESET TACHO\n\n");
//                             //spawner.motor_control(MotorControlEvent::ResetTacho).ok(); // TODO: count errors;
//                         }
//                         //crc_framer::CrcFramerSer::commit_frame(frame, bbbuffer_p);
//                         //rtic::pend(VESC_IRQ_EXTI);
//                     });
//                 },
//                 _ => {}
//             }
//         } else if #[cfg(feature = "br")] {
//             use embedded_hal::serial::Read;
//             use rtt_target::{rprint, rprintln};
//             use crate::rplidar;
//             let lidar = cx.resources.lidar;
//             match cx.resources.lidar_dma_c.read() {
//                 Ok(rgr) => {
//                     let len = rgr.len();
//                     //rprintln!(=>5, "Eat {} from DMA\n", len);
//                     for byte in rgr.iter() {
//                         lidar.eat_byte(*byte,
//                             |tx_bytes| {
//                                 rprint!(=>5, "ctrl_irq:tx_dma:[");
//                                 for b in tx_bytes {
//                                     rprint!(=>5, "{:02x} ", b);
//                                 }
//                                 rprintln!(=>5, "]\n");
//                                 match bbbuffer_p.grant_exact(tx_bytes.len()) {
//                                     Ok(mut wgr) => {
//                                         wgr.copy_from_slice(tx_bytes);
//                                         wgr.commit(tx_bytes.len());
//                                         rtic::pend(config::CTRL_TX_DMA);
//                                     },
//                                     Err(_) => {
//                                         rprintln!(=>5, "failed");
//                                     }
//                                 }
//                                 rprintln!(=>5, "{}DMA disable\n{}", color::CYAN, color::DEFAULT);
//
//                                 unsafe {
//                                     let dma1 = &(*hal::stm32::DMA1::ptr());
//                                     let stream5 = &dma1.st[5];
//                                     stream5.cr.modify(|_, w| w.en().disabled());
//                                 }
//                                 use hal::serial::Event;
//                                 serial.listen(Event::Rxne);
//                             });
//                     }
//                     rgr.release(len);
//                 },
//                 Err(_) => {}
//             }
//             match serial.read() {
//                 Ok(byte) => {
//                     //let spawner = cx.spawn;
//
//                     lidar.eat_byte(byte,
//                         |tx_bytes| {
//                             rprint!(=>5, "ctrl_irq:tx:[");
//                             for b in tx_bytes {
//                                 rprint!(=>5, "{:02x} ", b);
//                             }
//                             rprintln!(=>5, "]\n");
//                             match bbbuffer_p.grant_exact(tx_bytes.len()) {
//                                 Ok(mut wgr) => {
//                                     wgr.copy_from_slice(tx_bytes);
//                                     wgr.commit(tx_bytes.len());
//                                     rtic::pend(config::CTRL_TX_DMA);
//                                 },
//                                 Err(_) => {
//                                     rprintln!(=>5, "failed");
//                                 }
//                             }
//                         }
//                     );
//
//                     if lidar.state() == rplidar::LidarState::ReceivingScans {
//                         rprintln!(=>5, "{}DMA enable\n{}", color::CYAN, color::DEFAULT);
//
//                         unsafe {
//                             let dma1 = &(*hal::stm32::DMA1::ptr());
//                             let stream5 = &dma1.st[5];
//                             stream5.cr.modify(|_, w| w.en().enabled());
//                         }
//                         use hal::serial::Event;
//                         serial.unlisten(Event::Rxne);
//                     }
//                 },
//                 Err(e) => { }
//             }
//         }
//     }
// }

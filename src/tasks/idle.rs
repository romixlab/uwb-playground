use core::num::Wrapping;
use rtic::Mutex;
use rtt_target::{rprint, rprintln};
use crate::color;
use no_std_compat::prelude::v1::*;
use btoi::{btoi, ParseIntegerError};
use crate::radio::types::{RadioConfig, DummyMessage};
use dw1000::mac::{Address, AddressMode};
use dw1000::configs::UwbChannel;
use embedded_hal::serial::{Read, Write};
use crate::tasks::canbus::{ForwardHeap, ForwardEntry, Destination};
use vhrdcan::{FramePool, FrameId};
use crate::newconfig;
use embedded_hal::watchdog::Watchdog;
use cfg_if::cfg_if;

pub fn idle(mut cx: crate::idle::Context) -> ! {
    rprint!(=>0, "{}> {}", color::GREEN, color::DEFAULT);
    loop {
        let mut rtt_down = [0u8; 128];
        let rtt_down_len = cx.resources.rtt_down_channel.read(&mut rtt_down);
        if rtt_down_len > 0 {
            //let mut space_idxs = [0u8; 8];
            let args = core::str::from_utf8(&rtt_down);
            match args {
                Ok(args) => {
                    let mut args = args.split_ascii_whitespace();
                    let cmd = args.next();
                    if cmd.is_some() {
                        let cmd = cmd.unwrap();
                        match cmd {
                            "help" => {
                                rprintln!(=>0, "cmd [arg] [arg]...");
                                rprintln!(=>0, "Available commands: reset, uwb_adelay, analyzer");
                            },
                            "reset" => {
                                panic!("Reset requested");
                            },
                            "uwb_adelay" => {
                                uwb_antenna_delay_command(&mut args, &mut cx.resources.radio_commands);
                            },
                            "uwb_send" => {
                                uwb_send_command(&mut args, &mut cx.resources.radio_commands);
                            },
                            "uwb_listen" => {
                                uwb_listen(&mut args, &mut cx.resources.radio_commands);
                            },
                            "analyzer" => {
                                let _ = cx.spawn.can_analyzer(crate::tasks::canbus::CanAnalyzerEvent::Reset);
                            },
                            _ => {
                                rprintln!(=>0, "{}Unknown command, ignoring.{}", color::YELLOW, color::DEFAULT);
                            }
                        }
                    }
                },
                Err(_) => {
                    rprintln!(=>0, "{}Non-UTF8 command, ignoring.{}", color::YELLOW, color::DEFAULT);
                }
            }

            rprint!(=>0, "{}> {}", color::GREEN, color::DEFAULT);
        }

        match cx.resources.imx_serial.read() {
            Ok(c) => {
                rprint!(=>3, "imx: {}", c as char);
            },
            _ => {}
        }
        cx.resources.imx_serial.write(0xaa);
        cx.resources.imx_serial.flush();

        cfg_if! {
            if #[cfg(feature = "tof")] {
                let tof_mes = cx.resources.vl53l1_multi.read_all();
                rprintln!(=>4, "{:#?} mm\n\n", tof_mes);
                let mut tof_data = [0u8; 8];
                for i in 0..=1 {
                    tof_data[i*2..=(i*2+1)].copy_from_slice(&tof_mes.to_be_bytes());
                }

                let r = cx.resources.channels.lock(|channels| {
                    let forward_heap: &mut ForwardHeap = &mut channels.can0_forward_heap;
                    let forward_pool: &mut vhrdcan::FramePool = &mut channels.can0_forward_pool;
                    let frame = forward_pool.new_frame(FrameId::new_extended(newconfig::TOF_CAN_ID).unwrap(), &tof_data).unwrap();
                    let forward_entry = ForwardEntry {
                        to: Destination::Broadcast,
                        frame
                    };
                    forward_heap.push(forward_entry)
                });
                if r.is_err() {
                    rprintln!(=>4, "{}Heap full!{}", color::YELLOW, color::DEFAULT);
                }
            }
        }


        cx.resources.idle_counter.lock(|counter| *counter += Wrapping(1u32));
        //cortex_m::asm::delay(1_000_000);
        //atomic::compiler_fence(Ordering::SeqCst);
    }
}

fn uwb_antenna_delay_command(args: &mut core::str::SplitAsciiWhitespace, radio_commands_p: &mut crate::resources::radio_commands) {
    use crate::radio::types::Command;
    let tx_delay = args.next();
    let rx_delay = args.next();
    match (tx_delay, rx_delay) {
        (Some(tx_delay), Some(rx_delay)) => {
            let tx_delay: Result<u16, ParseIntegerError> = btoi(tx_delay.as_bytes());
            let rx_delay: Result<u16, ParseIntegerError> = btoi(rx_delay.as_bytes());
            match (tx_delay, rx_delay) {
                (Ok(tx_delay), Ok(rx_delay)) => {
                    let result = radio_commands_p.lock(|radio_commands_p| {
                        radio_commands_p.enqueue(Command::SetAntennaDelay(tx_delay, rx_delay))
                    });
                    rprintln!(=>0, "Setting delay to: tx: {} rx: {} ok:{}", tx_delay, rx_delay, result.is_ok());
                },
                _ => {
                    rprintln!(=>0, "{}Wrong numbers, should be ticks in dec, <= 65535{}", color::RED, color::DEFAULT);
                }
            }
        },
        _ => {
            rprintln!(=>0, "{}Wrong syntax, provide tx and rx delay{}", color::RED, color::DEFAULT);
        }
    }
}

fn uwb_send_command(args: &mut core::str::SplitAsciiWhitespace, radio_commands_p: &mut crate::resources::radio_commands) {
    use crate::radio::types::Command;
    let mut radio_config = RadioConfig::default();
    radio_config.channel = UwbChannel::Channel3;

    let result = radio_commands_p.lock(|radio_commands_p| {
        radio_commands_p.enqueue(Command::SendMessage(radio_config, Address::broadcast(&AddressMode::Short), DummyMessage{length: 16}))
    });
    rprintln!(=>0, "Sending: {}", result.is_ok());
}

fn uwb_listen(args: &mut core::str::SplitAsciiWhitespace, radio_commands_p: &mut crate::resources::radio_commands) {
    use crate::radio::types::Command;
    let  mut radio_config = RadioConfig::default();
    radio_config.channel = UwbChannel::Channel3;

    let result = radio_commands_p.lock(|radio_commands_p| {
        radio_commands_p.enqueue(Command::Listen(radio_config))
    });
    rprintln!(=>0, "Listen: {}", result.is_ok());
}
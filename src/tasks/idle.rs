use core::num::Wrapping;
use rtic::Mutex;
use rtt_target::{rprint, rprintln};
use crate::color;
use no_std_compat::prelude::v1::*;
use btoi::{btoi, ParseIntegerError};

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
                                rprintln!(=>0, "Available commands: uwb_adelay");
                            },
                            "uwb_adelay" => {
                                uwb_antenna_delay_command(&mut args, &mut cx.resources.radio_commands);
                            },
                            "uwb_send" => {
                                uwb_send_command(&mut args, &mut cx.resources.radio_commands);
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

        cx.resources.idle_counter.lock(|counter| *counter += Wrapping(1u32));
        cortex_m::asm::delay(20_000_000);
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

}
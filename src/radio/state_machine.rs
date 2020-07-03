use super::{
    RadioState,
    ReadyRadio, SendingRadio, ReceivingRadio
};
use rtt_target::{rprintln};
use dw1000::{
    mac,
    configs::{
        TxConfig,
        RxConfig,
        SfdSequence,
        BitRate,
        PreambleLength,
        UwbChannel
    },
};

use embedded_hal::digital::v2::OutputPin;
//use embedded_hal::blocking::spi;
use crate::config;
//use heapless::spsc::Consumer;
//use heapless::consts::*;
use crate::radio::message::TxMessage;
use crate::radio::Dw1000Spi;
use crate::config::Dw1000Cs;

pub fn advance<TP>(
    radio_state: &mut RadioState,
    commands: &mut super::CommandQueueC,
    rx_buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
    trace_pin: &mut TP)
    where TP: OutputPin
{
    use RadioState::*;
    const SM_FAIL_MESSAGE: &'static str = "Radio state machine fail";

    *radio_state = match radio_state {
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_ready(ready_radio, commands, trace_pin)
        },
        #[cfg(feature = "master")]
        GTSStartSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio, trace_pin)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            let answers_received = sd.1;
            advance_gts_answers_receiving(receiving_radio, answers_received, rx_buffer, commands, spawn, trace_pin)
        }
        #[cfg(feature = "slave")]
        GTSStartWaiting(sd) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, rx_buffer, spawn, trace_pin)
        }
        #[cfg(feature = "slave")]
        GTSWaitingForUplinkData(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_waiting_for_uplink_data(ready_radio, rx_buffer, commands)
        }
        #[cfg(feature = "slave")]
        GTSAnswerSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answer_sending(sending_radio, trace_pin)
        }
    };
}

fn get_txconfig() -> TxConfig {
    let bitrate = BitRate::Kbps850;
    let preamble_length = PreambleLength::Symbols512;
    let sfd_sequence = SfdSequence::DecawaveAlt;
    let channel = UwbChannel::Channel5;
    TxConfig {
        channel,
        bitrate,
        preamble_length,
        sfd_sequence,
        ..TxConfig::default()
    }
}

#[cfg(feature = "master")]
fn advance_ready<TP>(mut ready_radio: ReadyRadio, commands: &mut super::CommandQueueC, trace_pin: &mut TP) -> RadioState
    where TP: OutputPin
{
    use super::Command::*;
    match commands.dequeue() {
        Some(GTSStart(gts_start)) => {
            let tx_message = TxMessage {
                recipient: mac::Address::broadcast(&mac::AddressMode::Short),
                tx_time: None,
                payload: gts_start
            };
            let tx_config = get_txconfig();
            trace_pin.set_high().ok();
            ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
            let sending_radio = tx_message.send(ready_radio, tx_config).expect("DW1000 internal failure?");
            RadioState::GTSStartSending(Some(sending_radio))
        },
        _ => {
            RadioState::Ready(Some(ready_radio))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_start_sending<TP>(mut sending_radio: SendingRadio, trace_pin: &mut TP) -> RadioState
    where TP: OutputPin
{
    match sending_radio.wait() {
        Ok(_) => {
            trace_pin.set_low().ok();
            rprintln!(=>1,"\nGTSStart sent, listening...");
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, trace_pin);
            trace_pin.set_high().ok();
            RadioState::GTSAnswersReceiving((Some(receiving_radio), 0))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSStartSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"GTSStart send error: {:?}", e);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                trace_pin.set_low().ok();
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

#[cfg(feature = "master")]
fn process_messages_gts_answers_receiving<TP>(
    receiving_radio: ReceivingRadio,
    message: dw1000::hl::Message,
    answers_received: u8,
    spawn: &crate::radio_irq::Spawn,
    trace_pin: &mut TP
) -> RadioState
    where TP: OutputPin
{
    use super::message::{GTSAnswer};
    use dw1000::ranging::{Message, RxMessage};

    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    //let frame = message.frame;
    let gts_answer: Result<Option<RxMessage<GTSAnswer>>, dw1000::hl::Error<Dw1000Spi, Dw1000Cs>> = GTSAnswer::decode(&message);
    match gts_answer {
        Ok(gts_answer) => {
            match gts_answer {
                Some(gts_answer) => {
                    //rprintln!(=>1,"GTSAnswer: tacho: {} pwr_in: {}", gts_answer.payload.data.tacho, gts_answer.payload.data.power_in);
                    if let dw1000::mac::Address::Short(_, src) = message.frame.header.source {
                        spawn.radio_event(super::Event::GTSAnswerReceived(src, gts_answer.payload)).ok(); // TODO: count errors
                    }
                    // Continue listening for GTSAnswer's
                    let answers_received = answers_received + 1;
                    if answers_received == config::REQUIRED_SLAVE_COUNT {
                        spawn.radio_event(super::Event::GTSEnded).ok(); // TODO: count errors;
                        RadioState::Ready(Some(ready_radio))
                    } else {
                        let receiving_radio = enable_receiver(ready_radio, trace_pin);
                        RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
                    }

                },
                None => {
                    rprintln!(=>1,"Unkown message"); // TODO: Process other messages
                    let receiving_radio = enable_receiver(ready_radio, trace_pin);
                    RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
                }
            }
        },
        Err(e) => {
            rprintln!(=>1,"Message decode error: {:?}", e);
            let receiving_radio = enable_receiver(ready_radio, trace_pin);
            RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving<TP>(
    mut receiving_radio: ReceivingRadio,
    answers_received: u8,
    rx_buffer: &mut[u8],
    commands: &mut super::CommandQueueC,
    spawn: &crate::radio_irq::Spawn,
    trace_pin: &mut TP
) -> RadioState
    where TP: OutputPin
{
    use super::Command::*;
    match commands.peek() {
        Some(cmd) => {
            match cmd {
                //  ┌─ One or more GTS answer is missing, ignore and continue, should not happen?
                //  │            ┌─ Valid GTS termination from radio_chrono
                GTSStart(_) | GTSEnd => {
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    spawn.radio_event(super::Event::GTSEnded).ok(); // TODO: count errors
                    return advance_ready(ready_radio, commands, trace_pin);
                }
            }
        },
        _ => { }
    }
    match receiving_radio.wait(rx_buffer) {
        Ok(message) => {
            trace_pin.set_low().ok();
            process_messages_gts_answers_receiving(receiving_radio, message.0, answers_received, spawn, trace_pin)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                //rprintln!(=>1,"blockon:{:?}", e);
                //let sys_status = receiving_radio.ll().sys_status().read();
                //rprintln!(=>1,"sstY {:?}", sys_status);
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
            } else { // Actuall error while receiving
            rprintln!(=>1,"GTSAnswer receive error: {:?}", e);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, trace_pin);
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
            }
        }
    }
}

fn enable_receiver<TP>(mut ready_radio: ReadyRadio, _trace_pin: &mut TP) -> ReceivingRadio
    where TP: OutputPin
{
    let bitrate = BitRate::Kbps850;
    let preamble_length = PreambleLength::Symbols512;
    let sfd_sequence = SfdSequence::DecawaveAlt;
    let channel = UwbChannel::Channel5;
    let rx_config = RxConfig {
        channel,
        bitrate,
        expected_preamble_length: preamble_length,
        sfd_sequence,
        frame_filtering: false,
        ..RxConfig::default()
    };
    ready_radio.enable_rx_interrupts().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    ready_radio.receive(rx_config).expect("dw1000 crate or spi failure") // TODO: try to re-init and recover
}

#[cfg(feature = "slave")]
fn advance_ready<TP>(ready_radio: ReadyRadio, _commands: &mut super::CommandQueueC, trace_pin: &mut TP) -> RadioState
    where TP: OutputPin
{
    let receiving_radio = enable_receiver(ready_radio, trace_pin);
    RadioState::GTSStartWaiting(Some(receiving_radio))
}

#[cfg(feature = "slave")]
fn process_messages_gts_start_waiting<TP>(
    receiving_radio: ReceivingRadio,
    message: dw1000::hl::Message,
    spawn: &crate::radio_irq::Spawn,
    trace_pin: &mut TP
) -> RadioState
    where TP: OutputPin
{
    use super::message::{GTSStart, GTSUplinkData, GTSAnswer};
    use dw1000::ranging::{Message, RxMessage};
    use dw1000::time::Duration;

    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    //let frame = message.frame;
    let gts_start: Result<Option<RxMessage<GTSStart>>, dw1000::hl::Error<Dw1000Spi, Dw1000Cs>> = GTSStart::decode(&message);
    match gts_start {
        Ok(gts_start) => {
            match gts_start {
                Some(gts_start) => {
                    //rprintln!(=>1,"dest:{:?}\tgts_start:{:?}", frame.header.destination, gts_start);
                    //rprintln!(=>1,"GTSStart: dt: {} window: {} rpm: {}", timeslot.delta, timeslot.window, downlink_data.rpm);
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "devnode")] {
                            rprintln!(=> 1, "GTSStart");
                            let receiving_radio = enable_receiver(ready_radio, trace_pin);
                            return RadioState::GTSStartWaiting(Some(receiving_radio));
                        }
                    }

                    let timeslot = &gts_start.payload.timeslots[config::SLAVE_ID];
                    //let downlink_data = timeslot.sync_no_ack_data;
                    let tx_time = if timeslot.delta == 0 {
                        None
                    } else {
                        Some(gts_start.rx_time + Duration::from_nanos(timeslot.delta as u32 * 1_000))
                    };
                    let last_timeslot = &gts_start.payload.timeslots[(config::REQUIRED_SLAVE_COUNT - 1) as usize];
                    let gts_end_dt = super::NanoSeconds(last_timeslot.delta as u32 + last_timeslot.window as u32 + 2_000);
                    spawn.radio_event(super::Event::GTSStartReceived(tx_time, gts_end_dt, *timeslot)).ok(); // TODO: count errors
                    RadioState::GTSWaitingForUplinkData(Some(ready_radio))
                },
                None => {
                    rprintln!(=>1,"Unkown message");
                    let receiving_radio = enable_receiver(ready_radio, trace_pin);
                    RadioState::GTSStartWaiting(Some(receiving_radio))
                }
            }
        },
        Err(e) => {
            rprintln!(=>1,"Message decode error: {:?}", e);
            let receiving_radio = enable_receiver(ready_radio, trace_pin);
            RadioState::GTSStartWaiting(Some(receiving_radio))
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_start_waiting<TP>(
    mut receiving_radio: ReceivingRadio,
    rx_buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
    trace_pin: &mut TP
) -> RadioState
    where TP: OutputPin
{
    //let sys_status = receiving_radio.ll().sys_status().read().unwrap();
    //rprintln!(=>1,"A: {}", sys_status);

    match receiving_radio.wait(rx_buffer) {
        Ok((message, sys_status_before)) => {
            //trace_pin.set_high().ok();
            //rprintln!(=>1,"C: {}", sys_status_before);
            //let sys_status_after = receiving_radio.ll().sys_status().read().unwrap();
            //rprintln!(=>1,"D: {}", sys_status_after);
            process_messages_gts_start_waiting(receiving_radio, message, spawn, trace_pin)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                //rprintln!(=>1,"blockon:{:?}", e);
                let sys_status = receiving_radio.ll().sys_status().read().unwrap();
                rprintln!(=>1,"B: {}", sys_status);
                rprintln!(=>1, "B");
                cortex_m::asm::delay(1000);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!(=>1,"RX error: {:?}", e);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, trace_pin);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_waiting_for_uplink_data(
    mut ready_radio: ReadyRadio,
    _rx_buffer: &mut[u8],
    commands: &mut super::CommandQueueC,
) -> RadioState
{
    use super::Command::*;
    use dw1000::time::Duration;
    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            let receiving_radio = enable_receiver(ready_radio, trace_pin);
            return RadioState::GTSStartWaiting(Some(receiving_radio));
        }
    }
    loop {
        match commands.dequeue() {
            Some(cmd) => {
                match cmd {
                    GTSSendAnswer(instant, gts_answer) => {
                        let tx_message = TxMessage {
                            recipient: mac::Address::broadcast(&mac::AddressMode::Short), // TODO: Send to PAN, not broadcast
                            tx_time: instant,
                            payload: gts_answer
                        };
                        let tx_config = get_txconfig();
                        ready_radio.enable_tx_interrupts().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                        let sending_radio = tx_message.send(ready_radio, tx_config).expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                        return RadioState::GTSAnswerSending(Some(sending_radio))
                    },
                    _ => {
                        rprintln!(=> 1, "eaten other cmd!");
                        continue;
                    } // TODO: Do NOT eat other commands?
                }
            },
            _ => { return RadioState::GTSWaitingForUplinkData(Some(ready_radio)); }
        }
    }

}

#[cfg(feature = "slave")]
fn advance_gts_answer_sending<TP>(mut sending_radio: SendingRadio, trace_pin: &mut TP) -> RadioState
    where TP: OutputPin
{
    match sending_radio.wait() {
        Ok(_) => {
            rprintln!(=>1,"GTS answer sent");
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            trace_pin.set_high().ok();
            cortex_m::asm::delay(30);
            trace_pin.set_low().ok();
            // Switch back to GTSStart waiting state
            let receiving_radio = enable_receiver(ready_radio, trace_pin);
            RadioState::GTSStartWaiting(Some(receiving_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // GTSAnswer is still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSAnswerSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"GTS ans send error: {:?}", e);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, trace_pin);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}
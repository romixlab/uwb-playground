use super::{
    RadioState,
    ReadyRadio, SendingRadio, ReceivingRadio
};
use rtt_target::{rprintln};
use dw1000::{
    configs::{
        TxConfig,
        RxConfig,
        SfdSequence,
        BitRate,
        PreambleLength,
        UwbChannel
    },
};
#[cfg(feature = "master")]
use dw1000::mac;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi;
use crate::config;
use heapless::spsc::Consumer;
use heapless::consts::*;
use crate::radio::message::TxMessage;

pub fn advance<TP>(radio_state: &mut RadioState, commands: &mut super::CommandQueueC, rx_buffer: &mut[u8], trace_pin: &mut TP)
    where TP: OutputPin
{
    use RadioState::*;
    const SM_FAIL_MESSAGE: &'static str = "Radio state machine fail";

    *radio_state = match radio_state {
        Ready(sd) => {
            let mut ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_ready(ready_radio, commands)
        },
        #[cfg(feature = "master")]
        GTSStartSending(sd) => {
            let mut sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving(sd) => {
            let mut receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            let answers_received = sd.1;
            advance_gts_answers_receiving(receiving_radio, answers_received)
        }
        #[cfg(feature = "slave")]
        GTSStartWaiting(sd) => {
            let mut receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, rx_buffer, trace_pin)
        }
        #[cfg(feature = "slave")]
        GTSAnswerSending(sd) => {
            let mut sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answer_sending(sending_radio)
        }
    };
}

#[cfg(feature = "master")]
fn advance_ready(mut ready_radio: ReadyRadio, commands: &mut super::CommandQueueC) -> RadioState {
    use super::Command::*;
    match commands.dequeue() {
        Some(GTSStart(gts_start)) => {
            let tx_message = TxMessage {
                recipient: mac::Address::broadcast(&mac::AddressMode::Short),
                tx_time: None,
                payload: gts_start
            };
            let bitrate = BitRate::Kbps850;
            let preamble_length = PreambleLength::Symbols512;
            let sfd_sequence = SfdSequence::DecawaveAlt;
            let channel = UwbChannel::Channel5;
            let tx_config = TxConfig {
                channel,
                bitrate,
                preamble_length,
                sfd_sequence,
                ..TxConfig::default()
            };
            let sending_radio = tx_message.send(ready_radio, tx_config).unwrap();
            RadioState::GTSStartSending(Some(sending_radio))
        },
        None => {
            RadioState::Ready(Some(ready_radio))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_start_sending(mut sending_radio: SendingRadio) -> RadioState {
    match sending_radio.wait() {
        Ok(_) => {
            rprintln!("TX ok");
            let ready_radio = sending_radio.finish_sending().unwrap();
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still sending
                rprintln!("blockon:{:?}", e);
                RadioState::GTSStartSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!("TX error: {:?}", e);
                let ready_radio = sending_radio.finish_sending().unwrap();
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving(mut receiving_radio: ReceivingRadio, answers_received: u8) -> RadioState {
    RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
}

#[cfg(feature = "slave")]
fn advance_ready(mut ready_radio: ReadyRadio, _commands: &mut super::CommandQueueC) -> RadioState {
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
    ready_radio.enable_rx_interrupts().unwrap();
    let receiving_radio = ready_radio.receive(rx_config).unwrap();
    RadioState::GTSStartWaiting(Some(receiving_radio))
}

#[cfg(feature = "slave")]
fn advance_gts_start_waiting<TP>(mut receiving_radio: ReceivingRadio, rx_buffer: &mut[u8], trace_pin: &mut TP) -> RadioState
    where TP: OutputPin
{
    //let sys_status = receiving_radio.ll().sys_status().read();
    //rprintln!("sstX {:?}", sys_status);
    match receiving_radio.wait(rx_buffer) {
        Ok(message) => {
            trace_pin.set_high().ok();
            let frame = message.frame;
            rprintln!("dest:{:?}\tsource:{:?}\tseq:{}\tdata:{:?}", frame.header.destination, frame.header.source, frame.header.seq, frame.payload);
            let ready_radio = receiving_radio.finish_receiving().unwrap();

            rtic::pend(config::DW1000_IRQ_EXTI); // hacky rx restart
            trace_pin.set_low().ok();
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                //rprintln!("blockon:{:?}", e);
                //let sys_status = receiving_radio.ll().sys_status().read();
                //rprintln!("sstY {:?}", sys_status);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!("RX error: {:?}", e);
                let ready_radio = receiving_radio.finish_receiving().unwrap();
                rtic::pend(config::DW1000_IRQ_EXTI); // hacky rx restart
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_answer_sending(mut sending_radio: SendingRadio) -> RadioState {
    RadioState::GTSAnswerSending(Some(sending_radio))
}
use super::{
    UWBRadio, DW1000State
};
use rtt_target::{rprintln};
use dw1000::{
    configs::{
        SfdSequence,
        BitRate,
        PreambleLength,
        UwbChannel
    },
};
#[cfg(feature = "master")]
use dw1000::mac;

use embedded_hal::digital::v2::OutputPin;
use crate::config;
use super::RadioState;

pub fn on_radio_event<TP>(uwb: &mut UWBRadio, radio_state: &mut RadioState, rx_buffer: &mut[u8], trace_pin: &mut TP)
    where TP: OutputPin
{
    trace_pin.set_high().ok();

    let bitrate = BitRate::Kbps850;
    let preamble_length = PreambleLength::Symbols512;
    let sfd_sequence = SfdSequence::DecawaveAlt;
    let channel = UwbChannel::Channel5;

    // cx.resources.uwb: &mut DW1000State<_, _>
    *uwb = match uwb {
        DW1000State::Ready(uwb) => {
            let mut ready_radio = uwb.take().expect("DW1000 state machine fail");
            //let sys_status = ready_radio.ll().sys_status().read();
            //rprint!("ready {:?}", sys_status);

            cfg_if::cfg_if! {
                if #[cfg(feature = "slave")] {
                    use dw1000::configs::RxConfig;
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
                    //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                    trace_pin.set_low().ok();
                    rprintln!("grx");
                    DW1000State::Receiving(Some(receiving_radio))
                } else if #[cfg(feature = "master")] {
                    use dw1000::configs::TxConfig;
                    let tx_config = TxConfig {
                        channel,
                        bitrate,
                        preamble_length,
                        sfd_sequence,
                        ..TxConfig::default()
                    };
                    ready_radio.enable_tx_interrupts().unwrap();
                    let sending_radio = ready_radio.send(b"ping", mac::Address::broadcast(&mac::AddressMode::Short), None, tx_config).unwrap();
                    //let sys_status = sending_radio.ll().sys_status().read();
                    //rprintln!(" {:?}", sys_status);
                    trace_pin.set_low().ok();
                    rprintln!("gtx");
                    DW1000State::Sending(Some(sending_radio))
                }
            }
        },
        DW1000State::Receiving(uwb) => {
            let mut receiving_radio = uwb.take().expect("DW1000 state machine fail");
            let sys_status = receiving_radio.ll().sys_status().read();
            rprintln!("sys_status {:?}", sys_status);

            match receiving_radio.wait(rx_buffer) {
                Ok(message) => {
                    let frame = message.frame;
                    rprintln!("dest:{:?}\tsource:{:?}\tseq:{}\tdata:{:?}", frame.header.destination, frame.header.source, frame.header.seq, frame.payload);
                    let ready_radio = receiving_radio.finish_receiving().unwrap();
                    //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                    trace_pin.set_low().ok();

                    rtic::pend(config::DW1000_IRQ_EXTI); // hacky rx restart
                    DW1000State::Ready(Some(ready_radio))
                },
                Err(e) => {
                    if let nb::Error::WouldBlock = e { // Still receiving
                    rprintln!("blockon:{:?}", e);
                        //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                        trace_pin.set_low().ok();
                        DW1000State::Receiving(Some(receiving_radio))
                    } else { // Actuall error while receiving
                    rprintln!("RX error: {:?}", e);
                        let ready_radio = receiving_radio.finish_receiving().unwrap();
                        //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                        trace_pin.set_low().ok();
                        rtic::pend(config::DW1000_IRQ_EXTI); // hacky rx restart
                        DW1000State::Ready(Some(ready_radio))
                    }
                }
            }
        },
        DW1000State::Sending(uwb) => {
            let mut sending_radio = uwb.take().expect("DW1000 state machine fail");
            //let sys_status = sending_radio.ll().sys_status().read().unwrap();
            //rprint!("sending {:?}", sys_status);
            match sending_radio.wait() {
                Ok(_) => {
                    rprintln!("TX ok");
                    let ready_radio = sending_radio.finish_sending().unwrap();
                    trace_pin.set_low().ok();
                    //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                    DW1000State::Ready(Some(ready_radio))
                },
                Err(e) => {
                    if let nb::Error::WouldBlock = e { // Still sending
                    rprintln!("blockon:{:?}", e);
                        trace_pin.set_low().ok();
                        //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                        DW1000State::Sending(Some(sending_radio))
                    } else { // Actuall error while sending
                    rprintln!("TX error: {:?}", e);
                        let ready_radio = sending_radio.finish_sending().unwrap();
                        trace_pin.set_low().ok();
                        //cx.resources.uwb_irq.clear_interrupt_pending_bit();
                        DW1000State::Ready(Some(ready_radio))
                    }
                }
            }
        }
    };
    //cx.resources.uwb_irq.clear_interrupt_pending_bit();
}
use super::{
    RadioState,
    ReadyRadio, SendingRadio, ReceivingRadio
};
use rtt_target::{ rprint, rprintln };
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
use crate::radio::{Dw1000Spi, Arbiter, MiniMultiplexer};
use crate::config::Dw1000Cs;
use crate::util::{Tracer, TraceEvent};

pub fn advance<A: Arbiter, T: Tracer>(
    radio_state: &mut RadioState,
    arbiter: &mut A,
    commands: &mut super::CommandQueueC,
    buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
    tracer: &mut T
) {
    use RadioState::*;
    const SM_FAIL_MESSAGE: &'static str = "Radio state machine fail";

    *radio_state = match radio_state {
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_ready(ready_radio, arbiter, tracer, buffer, commands)
        },
        #[cfg(feature = "master")]
        GTSStartSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio, arbiter, tracer)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            let answers_received = sd.1;
            advance_gts_answers_receiving(receiving_radio, arbiter, tracer, answers_received, buffer, commands, spawn)
        }
        #[cfg(feature = "slave")]
        GTSStartWaiting(sd) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, arbiter, tracer, buffer, spawn)
        }
        #[cfg(feature = "slave")]
        GTSWaitingForUplinkData(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_waiting_for_uplink_data(ready_radio, arbiter, tracer, buffer, commands)
        }
        #[cfg(feature = "slave")]
        GTSAnswerSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answer_sending(sending_radio, arbiter, tracer)
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
fn advance_ready<A: Arbiter, T: Tracer>(
    mut ready_radio: ReadyRadio,
    arbiter: &mut A,
    tracer: &mut T,
    buffer: &mut[u8],
    commands: &mut super::CommandQueueC,
) -> RadioState
{
    use super::Command::*;
    match commands.dequeue() {
        Some(GTSStart) => {
            use dw1000::mac;
            use dw1000::mac::{Frame, Header};
            let frame = mac::Frame {
                header: mac::Header {
                    frame_type:      mac::FrameType::Data,
                    version:         mac::FrameVersion::Ieee802154_2006,
                    security:        mac::Security::None,
                    frame_pending:   false,
                    ack_request:     false,
                    pan_id_compress: false,
                    destination:     mac::Address::broadcast(&mac::AddressMode::Short),
                    source:          mac::Address::Short(config::PAN_ID, config::UWB_ADDR),
                    seq:             0,
                },
                content: mac::FrameContent::Data,
                payload: &[],
                footer: [0; 2],
            };
            //seq += Wrapping(1u8);
            let mut len = frame.encode(buffer, mac::WriteFooter::No);

            use crate::radio::serdes::BufMut;
            let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
            let mut mux = MiniMultiplexer::new(bufmut);

            let mut window = crate::radio::message::Window {
                shift: crate::units::MicroSeconds(0x1234),
                window: crate::units::MicroSeconds(0x5678),
                window_type: crate::radio::message::WindowType::Uplink,
                radio_config: crate::radio::message::RadioConfig {
                    channel: dw1000::configs::UwbChannel::Channel5,
                    bitrate: dw1000::configs::BitRate::Kbps850,
                    prf: dw1000::configs::PulseRepetitionFrequency::Mhz64
                }
            };
            let r = mux.mux(&window, super::LogicalDestination::Implicit, super::ChannelId::new(0));
            rprintln!(=>1, "mux1:{:?}", r);

            window.shift = crate::units::MicroSeconds(0xaabb);
            window.window = crate::units::MicroSeconds(0xccdd);
            mux.mux(&window, super::LogicalDestination::Unicast(dw1000::mac::ShortAddress(0xeeff)), super::ChannelId::new(1));

            window.shift = crate::units::MicroSeconds(0x7766);
            window.window = crate::units::MicroSeconds(0x8899);
            let r = mux.mux(&window, super::LogicalDestination::Implicit, super::ChannelId::new(1));

            let r = mux.mux(&window, super::LogicalDestination::Unicast(config::BR_UWB_ADDR), super::ChannelId::new(0));

            arbiter.source_sync(&mut mux);
            use crate::radio::Multiplex;
            let bufmut_taken = mux.take();
            len += bufmut_taken.written();

            rprint!(=>1, "mux:[");
            for b in &buffer[0..len] {
                rprint!(=>1, "{:02x} ", b);
            }
            rprintln!(=>1, "]\n");

            ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
            let tx_config = get_txconfig();
            let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
            RadioState::GTSStartSending(Some(sending_radio))
        },
        _ => {
            RadioState::Ready(Some(ready_radio))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_start_sending<A: Arbiter, T: Tracer>(
    mut sending_radio: SendingRadio,
    arbiter: &mut A,
    tracer: &mut T,
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            tracer.event(TraceEvent::GTSStart);
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio);
            tracer.event(TraceEvent::GTSStartedReceivingAnswers);
            RadioState::GTSAnswersReceiving((Some(receiving_radio), 0))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSStartSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"GTSStart send error: {:?}", e);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                //tracer.event(TraceEvent::XYZ);
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

#[cfg(feature = "master")]
fn process_messages_gts_answers_receiving<A: Arbiter, T: Tracer>(
    receiving_radio: ReceivingRadio,
    arbiter: &mut A,
    tracer: &mut T,
    message: dw1000::hl::Message,
    answers_received: u8,
    spawn: &crate::radio_irq::Spawn,
) -> RadioState
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
                        let receiving_radio = enable_receiver(ready_radio);
                        RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
                    }

                },
                None => {
                    rprintln!(=>1,"Unkown message"); // TODO: Process other messages
                    let receiving_radio = enable_receiver(ready_radio);
                    RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
                }
            }
        },
        Err(e) => {
            rprintln!(=>1,"Message decode error: {:?}", e);
            let receiving_radio = enable_receiver(ready_radio);
            RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving<A: Arbiter, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    arbiter: &mut A,
    tracer: &mut T,
    answers_received: u8,
    buffer: &mut[u8],
    commands: &mut super::CommandQueueC,
    spawn: &crate::radio_irq::Spawn,
) -> RadioState
{
    use super::Command::*;
    match commands.peek() {
        Some(cmd) => {
            match cmd {
                //  ┌─ One or more GTS answer is missing, ignore and continue, should not happen?
                //  │            ┌─ Valid GTS termination from radio_chrono
                GTSStart | GTSEnd => {
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    spawn.radio_event(super::Event::GTSEnded).ok(); // TODO: count errors
                    tracer.event(TraceEvent::GTSEnded);
                    return advance_ready(ready_radio, arbiter, tracer, buffer, commands);
                }
            }
        },
        _ => { }
    }
    match receiving_radio.wait(buffer) {
        Ok(message) => {
            //tracer.event(TraceEvent::XYZ);
            process_messages_gts_answers_receiving(receiving_radio, arbiter, tracer, message.0, answers_received, spawn)
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
                let receiving_radio = enable_receiver(ready_radio);
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
            }
        }
    }
}

fn enable_receiver(mut ready_radio: ReadyRadio) -> ReceivingRadio
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
fn advance_ready<A: Arbiter, T: Tracer>(
    ready_radio: ReadyRadio,
    arbiter: &mut A,
    tracer: &mut T,
    buffer: &mut[u8],
    _commands: &mut super::CommandQueueC,
) -> RadioState
{
    let receiving_radio = enable_receiver(ready_radio);
    RadioState::GTSStartWaiting(Some(receiving_radio))
}

#[cfg(feature = "slave")]
fn process_messages_gts_start_waiting<A: Arbiter, T: Tracer>(
    receiving_radio: ReceivingRadio,
    arbiter: &mut A,
    tracer: &mut T,
    message: dw1000::hl::Message,
    spawn: &crate::radio_irq::Spawn,
) -> RadioState
{
    use super::message::{GTSStart};
    use dw1000::ranging::{Message, RxMessage};
    use dw1000::time::Duration;

    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    let payload = message.frame.payload;
    rprint!(=>1, "p:[");
    for b in payload {
        rprint!(=>1, "{:02x} ", b);
    }
    rprintln!(=>1, "]\n");
    use crate::radio::serdes::Buf;
    use crate::radio::{Demultiplex, MiniDemultiplexer};
    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    demux.demux(config::UWB_ADDR, |channel_id, chunk| {
        rprint!(=>1, "->{}:[", channel_id);
        for b in chunk {
            rprint!(=>1, "{:02x} ", b);
        }
        rprintln!(=>1, "]\n");
    });

    let receiving_radio = enable_receiver(ready_radio);
    RadioState::GTSStartWaiting(Some(receiving_radio))
    // let gts_start: Result<Option<RxMessage<GTSStart>>, dw1000::hl::Error<Dw1000Spi, Dw1000Cs>> = GTSStart::decode(&message);
    // match gts_start {
    //     Ok(gts_start) => {
    //         match gts_start {
    //             Some(gts_start) => {
    //                 //rprintln!(=>1,"dest:{:?}\tgts_start:{:?}", frame.header.destination, gts_start);
    //                 //rprintln!(=>1,"GTSStart: dt: {} window: {} rpm: {}", timeslot.delta, timeslot.window, downlink_data.rpm);
    //                 cfg_if::cfg_if! {
    //                     if #[cfg(feature = "devnode")] {
    //
    //                         let receiving_radio = enable_receiver(ready_radio, tracer);
    //                         return RadioState::GTSStartWaiting(Some(receiving_radio));
    //                     }
    //                 }
    //
    //                 let timeslot = &gts_start.payload.timeslots[config::SLAVE_ID];
    //                 //let downlink_data = timeslot.sync_no_ack_data;
    //                 let tx_time = if timeslot.delta == 0 {
    //                     None
    //                 } else {
    //                     Some(gts_start.rx_time + Duration::from_nanos(timeslot.delta as u32 * 1_000))
    //                 };
    //                 let last_timeslot = &gts_start.payload.timeslots[(config::REQUIRED_SLAVE_COUNT - 1) as usize];
    //                 let gts_end_dt = super::NanoSeconds(last_timeslot.delta as u32 + last_timeslot.window as u32 + 2_000);
    //                 spawn.radio_event(super::Event::GTSStartReceived(tx_time, gts_end_dt, *timeslot)).ok(); // TODO: count errors
    //                 RadioState::GTSWaitingForUplinkData(Some(ready_radio))
    //             },
    //             None => {
    //                 rprintln!(=>1,"Unkown message");
    //                 let receiving_radio = enable_receiver(ready_radio);
    //                 RadioState::GTSStartWaiting(Some(receiving_radio))
    //             }
    //         }
    //     },
    //     Err(e) => {
    //         rprintln!(=>1,"Message decode error: {:?}", e);
    //         let receiving_radio = enable_receiver(ready_radio);
    //         RadioState::GTSStartWaiting(Some(receiving_radio))
    //     }
    // }
}

#[cfg(feature = "slave")]
fn advance_gts_start_waiting<A: Arbiter, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    arbiter: &mut A,
    tracer: &mut T,
    buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
) -> RadioState
{
    //let sys_status = receiving_radio.ll().sys_status().read().unwrap();
    //rprintln!(=>1,"A: {}", sys_status);

    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            ////tracer.event(TraceEvent::XYZ);
            //rprintln!(=>1,"C: {}", sys_status_before);
            //let sys_status_after = receiving_radio.ll().sys_status().read().unwrap();
            //rprintln!(=>1,"D: {}", sys_status_after);
            process_messages_gts_start_waiting(receiving_radio, arbiter, tracer, message, spawn)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                //rprintln!(=>1,"blockon:{:?}", e);
                let sys_status = receiving_radio.ll().sys_status().read().unwrap();
                //rprintln!(=>1,"B: {}", sys_status);
                //rprintln!(=>1, "B");
                cortex_m::asm::delay(1000);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!(=>1,"RX error: {:?}", e);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_waiting_for_uplink_data<A: Arbiter, T: Tracer>(
    mut ready_radio: ReadyRadio,
    _arbiter: &mut A,
    _tracer: &mut T,
    _buffer: &mut[u8],
    commands: &mut super::CommandQueueC,
) -> RadioState
{
    use super::Command::*;
    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            let receiving_radio = enable_receiver(ready_radio, tracer);
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
fn advance_gts_answer_sending<A: Arbiter, T: Tracer>(
    mut sending_radio: SendingRadio,
    _arbiter: &mut A,
    tracer: &mut T,
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            tracer.event(TraceEvent::GTSAnswerSent);
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            //tracer.event(TraceEvent::XYZ);
            cortex_m::asm::delay(30);
            //tracer.event(TraceEvent::XYZ);
            // Switch back to GTSStart waiting state
            let receiving_radio = enable_receiver(ready_radio);
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
                let receiving_radio = enable_receiver(ready_radio);
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}
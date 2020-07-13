use super::types::{
    Radio,
    RadioState,
    RadioConfig,
    ReadyRadio, SendingRadio, ReceivingRadio,
    Window, WindowType,
    Event,
    CommandQueueC,
    Node, NodeState,
    Error
};
use super::types::{
    Command
};
use super::channelization::{
    Arbiter,
    Multiplex, Demultiplex,
    MiniMultiplexer, MiniDemultiplexer,
    ChannelId,
    LogicalDestination,
};
use rtt_target::{
    rprint,
    rprintln
};
use dw1000::{
    mac,
    configs::{
        TxConfig,
        RxConfig,
        SfdSequence,
        BitRate,
        PreambleLength,
        UwbChannel,
        PulseRepetitionFrequency
    },
};
use crate::units::MicroSeconds;
use super::serdes::{
    Buf,
    BufMut,
    Serialize,
    Deserialize,
};
use super::scheduler::{
    Scheduler,
};
use rtic::cyccnt::Instant as CycntInstant;
use rtic::cyccnt::Duration as CycntDuration;
use rtic::cyccnt::U32Ext;
use dw1000::time::Instant as RadioInstant;
use dw1000::time::Duration as RadioDuration;
use cfg_if::cfg_if;
use embedded_hal::digital::v2::OutputPin;
use crate::config;

use crate::config::Dw1000Cs;
use crate::util::{Tracer, TraceEvent};

const SM_FAIL_MESSAGE: &'static str = "Radio state machine fail";

/// Convenient way to pass around all the fields below.
struct SMContext<'a, A, T> {
    arbiter: &'a mut A,
    tracer: &'a mut T,
    spawn: &'a crate::radio_irq::Spawn<'a>,
    schedule: &'a crate::radio_irq::Schedule<'a>,
    clocks: &'a crate::board::hal::rcc::Clocks,
    #[cfg(feature = "slave")]
    master_node: &'a mut NodeState,
    state_instant: &'a mut Option<CycntInstant>,
}

/// Get radio in the `Ready` state.
/// Radio is most likely already in the `Ready` state when calling this function.
/// Force finish receiving or sending, which is probably a bug, but not critical one.
fn prepare_radio(state: RadioState) -> ReadyRadio {
    use RadioState::*;
    match state {
        Ready(rs) => { rs.expect(SM_FAIL_MESSAGE) },
        #[cfg(feature = "master")]
        GTSStartSending(rs) => {
            let sending_radio = rs.expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "master")]
        GTSAnswersReceiving(rs) => {
            let receiving_radio = rs.0.expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "slave")]
        GTSStartWaiting(rs) => {
            let receiving_radio = rs.expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "slave")]
        GTSAnswerSending(rs) => {
            let sending_radio = rs.expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
    }
}

pub fn advance<A: Arbiter<Error = Error>, T: Tracer>(
    radio: &mut Radio,
    arbiter: &mut A,
    buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
    schedule: &crate::radio_irq::Schedule,
    clocks: &crate::board::hal::rcc::Clocks,
    tracer: &mut T
) {
    use RadioState::*;
    cfg_if! {
        if #[cfg(feature = "master")] {
            let cx = SMContext { arbiter, tracer, spawn, schedule, clocks, state_instant: &mut radio.state_instant };
        } else if #[cfg(feature = "slave")] {
            let cx = SMContext { arbiter, tracer, spawn, schedule, clocks, master_node: &mut radio.master, state_instant: &mut radio.state_instant };
        }
    }

    let commands = &mut radio.commands;
    match commands.peek() {
        Some(command) => {
            match command {
                Command::DynWindowStart => {
                    commands.dequeue();
                    cx.tracer.event(TraceEvent::DynWindowStart);
                },
                _ => {}
            }
        }
        _ => {}
    };
    let radio_state = &mut radio.state;
    *radio_state = match radio_state {
        #[cfg(feature = "master")]
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_ready(ready_radio, cx, buffer, commands)
        },
        #[cfg(feature = "master")]
        GTSStartSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio, cx)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            let answers_received = sd.1;
            advance_gts_answers_receiving(receiving_radio, cx, answers_received, buffer, commands)
        }
        #[cfg(feature = "slave")]
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_ready(ready_radio, arbiter, tracer, buffer, commands, &mut radio.master)
        },
        #[cfg(feature = "slave")]
        GTSStartWaiting(sd) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, cx, buffer)
        }
        #[cfg(feature = "slave")]
        GTSAnswerSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answer_sending(sending_radio, arbiter, tracer, spawn)
        }
    };
}

fn get_txconfig(radio_config: RadioConfig) -> TxConfig {
    TxConfig {
        channel: radio_config.channel,
        bitrate: radio_config.bitrate,
        preamble_length: radio_config.recommended_preamble(),
        sfd_sequence: radio_config.recommended_sfd(),
        pulse_repetition_frequency: radio_config.prf,
        ranging_enable: false
    }
}

fn enable_receiver(mut ready_radio: ReadyRadio, radio_config: RadioConfig) -> ReceivingRadio
{
    let rx_config = RxConfig {
        channel: radio_config.channel,
        bitrate: radio_config.bitrate,
        expected_preamble_length: radio_config.recommended_preamble(),
        sfd_sequence: radio_config.recommended_sfd(),
        frame_filtering: false,
        pulse_repetition_frequency: radio_config.prf,
    };
    ready_radio.enable_rx_interrupts().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    ready_radio.receive(rx_config).expect("dw1000 crate or spi failure") // TODO: try to re-init and recover
}

fn default_mac_frame(payload: &[u8]) -> mac::Frame {
    mac::Frame {
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
        payload,
        footer: [0; 2],
    }
}

#[cfg(feature = "master")]
fn advance_ready<A: Arbiter<Error = Error>, T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: SMContext<A, T>,
    buffer: &mut[u8],
    commands: &mut CommandQueueC,
) -> RadioState
{
    let command = commands.dequeue();
    if command.is_none() {
        return RadioState::Ready(Some(ready_radio));
    }
    match command.unwrap() {
        Command::GTSStart => {
            cx.tracer.event(TraceEvent::GTSStart);

            let frame = default_mac_frame(&[]);
            let mut len = frame.encode(buffer, mac::WriteFooter::No);

            let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
            let mut mux = MiniMultiplexer::new(bufmut);

            Scheduler::source_timeslots(&mut mux);
            cx.arbiter.source_sync(&mut mux);

            let bufmut_taken = mux.take();
            len += bufmut_taken.written();

            ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
            let tx_config = get_txconfig(RadioConfig::default());
            let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
            cx.tracer.event(TraceEvent::GTSStart);
            RadioState::GTSStartSending(Some(sending_radio))
        },
        // Command::Listen => {
        //
        // },
        _ => {
            RadioState::Ready(Some(ready_radio))
        }
    }
}

#[cfg(feature = "master")]
fn advance_gts_start_sending<A: Arbiter, T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: SMContext<A, T>
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::TimingMarker);
            let instant = CycntInstant::now();
            // Schedule sync period end
            let dt: MicroSeconds = Scheduler::sync_period_end_shift();
            cx.schedule.radio_event(instant + us2cycles!(cx.clocks, dt.0), Event::GTSAboutToEnd).ok();
            // Schedule dyn period start
            let dt: MicroSeconds = Scheduler::dyn_period_start_shift();
            cx.schedule.radio_event(instant + us2cycles!(cx.clocks, dt.0), Event::DynWindowAboutToStart).ok();

            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
            cx.tracer.event(TraceEvent::GTSStartedReceivingAnswers);
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
fn process_messages_gts_answers_receiving<A: Arbiter<Error = Error>, T: Tracer>(
    receiving_radio: ReceivingRadio,
    cx: SMContext<A, T>,
    message: dw1000::hl::Message,
    answers_received: u8,
) -> RadioState
{
    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    let payload = message.frame.payload;
    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    demux.demux(config::UWB_ADDR, |channel, chunk| {
        if channel.is_ctrl() {

        } else {
            cx.arbiter.sink_sync(channel, chunk);
        }
    });

    let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
    RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving<A: Arbiter<Error = Error>, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: SMContext<A, T>,
    answers_received: u8,
    buffer: &mut[u8],
    commands: &mut CommandQueueC,
) -> RadioState
{
    let command = commands.dequeue();
    if command.is_some() {
        use Command::*;
        match command.unwrap() {
            //  ┌─ One or more GTS answer is missing, ignore and continue, should not happen?
            //  │            ┌─ Valid GTS termination from radio_chrono
            GTSStart | GTSEnd => {
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                cx.tracer.event(TraceEvent::GTSEnded);
                return advance_ready(ready_radio, cx, buffer, commands);
            },
            _ => {}
        }
    }

    match receiving_radio.wait(buffer) {
        Ok(message) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            process_messages_gts_answers_receiving(receiving_radio, cx, message.0, answers_received)
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
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_ready<A: Arbiter<Error = super::Error>, T: Tracer>(
    ready_radio: ReadyRadio,
    arbiter: &mut A,
    tracer: &mut T,
    buffer: &mut[u8],
    commands: &mut CommandQueueC,
    master_node: &mut NodeState,
) -> RadioState
{
    match commands.dequeue() {
        Some(cmd) => {
            match cmd {
                Command::Listen(radio_config) => {
                    tracer.event(TraceEvent::Listening);
                    let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                    RadioState::GTSStartWaiting(Some(receiving_radio))
                },
                Command::SendGTSAnswer => {
                    tracer.event(TraceEvent::GTSAnswerSend);
                    send_gts_answer(ready_radio, arbiter, buffer, master_node)
                },
                _ => {
                    RadioState::Ready(Some(ready_radio))
                }
            }
        },
        None => { RadioState::Ready(Some(ready_radio)) }
    }
}

#[cfg(feature = "slave")]
fn send_gts_answer<A: Arbiter<Error = super::Error>>(
    mut ready_radio: ReadyRadio,
    arbiter: &mut A,
    buffer: &mut[u8],
    master_node: &mut NodeState,
) -> RadioState
{
    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
    let mut mux = MiniMultiplexer::new(bufmut);

    arbiter.source_sync(&mut mux);

    let bufmut_taken = mux.take();
    len += bufmut_taken.written();


    let tx_config = get_txconfig(RadioConfig::default());

    if let NodeState::Active(node) = master_node {
        let slot = node.slots[0].unwrap();
        let tx_time = if slot.shift.0 == 0 {
            None
        } else {
            Some(node.last_seen.unwrap().1 + RadioDuration::from_nanos(slot.shift.0 * 1_000))
        };
        ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
        let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
        RadioState::GTSAnswerSending(Some(sending_radio))
    } else {
        rprintln!(=>1, "\x1b[1;31;40msend_gts_answer: no slots");
        RadioState::Ready(Some(ready_radio))
    }
}

#[cfg(feature = "slave")]
fn process_messages_gts_start_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: SMContext<A, T>,
    message: dw1000::hl::Message,
) -> RadioState
{
    let cycnt_now = CycntInstant::now();
    let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    let payload = message.frame.payload;
    rprint!(=>1, "p:{}[", payload.len());
    // for b in payload {
    //     rprint!(=>1, "{:02x} ", b);
    // }
    rprintln!(=>1, "]\n");

    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    let mut slots_received = 0;
    let mut is_gts_start = false;
    demux.demux(config::UWB_ADDR, |channel, chunk| {
        if channel.is_ctrl() {
            //rprint!(=>1, "ctrl_ch:{}", chunk.len());
            let mut buf = Buf::new(&chunk);
            match Window::des(&mut buf) {
                Ok(window) => {
                    is_gts_start = true;
                    if slots_received == 0 {
                        *cx.master_node = NodeState::Active(Node{
                            last_seen: Some((cycnt_now, message.rx_time)),
                            slots: [Some(window), None, None]
                        });
                    } else {
                        if let NodeState::Active(node) = cx.master_node {
                            if slots_received < node.slots.len() {
                                node.slots[slots_received] = Some(window);
                            }
                        }
                    }
                    slots_received += 1;
                    //rprintln!(=>1, "{:?}", window);
                },
                Err(_) => {}
            }
        } else {
            cx.arbiter.sink_sync(channel, chunk);
        }
    });

    if is_gts_start {
        cx.tracer.event(TraceEvent::GTSStartReceived);

        // TODO: rx_time is 0 for some reason, need to figure out or ranging would not work
        // let processing_delay = ready_radio.sys_time().unwrap_or(message.rx_time).duration_since(message.rx_time);
        // rprintln!(=>13, "Mrt {}", message.rx_time.value() / 64);
        // rprintln!(=>13, "Pde {}", processing_delay.to_nanos());
        // let processing_delay = (cx.clocks.sysclk().0 / 1_000_000) * (processing_delay.to_nanos() / 1_000) as u32;
        let processing_delay = us2cycles_raw!(cx.clocks, 155);
        cx.spawn.radio_event(Event::GTSStartReceived(CycntDuration::from_cycles(processing_delay))).ok(); // TODO: count
    }

    RadioState::Ready(Some(ready_radio))

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
fn advance_gts_start_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: SMContext<A, T>,
    buffer: &mut[u8],
) -> RadioState
{
    //let sys_status = receiving_radio.ll().sys_status().read().unwrap();
    //rprintln!(=>1,"A: {}", sys_status);

    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            process_messages_gts_start_waiting(receiving_radio, cx, message)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                if cx.state_instant.is_none() {
                    *cx.state_instant = Some(CycntInstant::now());
                } else {
                    let elapsed = cx.state_instant.unwrap().elapsed().as_cycles();
                    if elapsed > 72_000_000 {
                        *cx.state_instant = Some(CycntInstant::now());
                        rprintln!(=>1,"ReListen");
                        let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure");
                        let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                        return RadioState::GTSStartWaiting(Some(receiving_radio));
                    }
                }
                RadioState::GTSStartWaiting(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!(=>1,"RX error: {:?}", e);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_answer_sending<A: Arbiter, T: Tracer>(
    mut sending_radio: SendingRadio,
    _arbiter: &mut A,
    tracer: &mut T,
    spawn: &crate::radio_irq::Spawn,

) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            tracer.event(TraceEvent::GTSAnswerSent);
            spawn.radio_event(Event::GTSAnswerSent).ok();
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // GTSAnswer is still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSAnswerSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"GTS ans send error: {:?}", e);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}
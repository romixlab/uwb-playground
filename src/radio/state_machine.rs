use super::types::{
    Radio,
    RadioState,
    RadioConfig,
    ReadyRadio, SendingRadio, ReceivingRadio,
    Slot, SlotType,
    Event,
    CommandQueueC,
    Node, NodeState,
    Error
};
use super::types::{
    Command,
    Pong
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
use crate::color;
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
    scheduler: &'a mut Scheduler,
    #[cfg(feature = "slave")]
    master_node: &'a mut NodeState,
    state_instant: &'a mut Option<CycntInstant>,
}

/// Get radio in the `Ready` state.
/// Radio is most likely already in the `Ready` state when calling this function.
/// Force finish receiving or sending, which is probably a bug, but not critical one.
/// Radio instance is `take()`'en from the state machine, make sure to put it back!
fn prepare_radio(state: &mut RadioState) -> ReadyRadio {
    use RadioState::*;
    let ready_radio = match state {
        Ready(rs) => { rs.take().expect(SM_FAIL_MESSAGE) },
        #[cfg(feature = "master")]
        GTSStartSending(rs) => {
            let sending_radio = rs.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "master")]
        GTSAnswersReceiving(rs) => {
            let receiving_radio = rs.0.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "slave")]
        GTSStartWaiting(rs) => {
            let receiving_radio = rs.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "slave")]
        GTSAnswerSending(rs) => {
            let sending_radio = rs.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
        DynReceiving(rs) => {
            let receiving_radio = rs.0.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        DynSending(rs) => {
            let sending_radio = rs.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        }
    };
    ready_radio
}

pub fn advance<A: Arbiter<Error = Error>, T: Tracer>(
    radio: &mut Radio,
    arbiter: &mut A,
    buffer: &mut[u8],
    spawn: &crate::radio_irq::Spawn,
    schedule: &crate::radio_irq::Schedule,
    clocks: &crate::board::hal::rcc::Clocks,
    scheduler: &mut Scheduler,
    tracer: &mut T
) {
    use RadioState::*;
    cfg_if! {
        if #[cfg(feature = "master")] {
            let mut cx = SMContext { arbiter, tracer, spawn, schedule, clocks, scheduler, state_instant: &mut radio.state_instant };
        } else if #[cfg(feature = "slave")] {
            let mut cx = SMContext { arbiter, tracer, spawn, schedule, clocks, scheduler, master_node: &mut radio.master, state_instant: &mut radio.state_instant };
        }
    }

    let commands = &mut radio.commands;
    let mut commands_processed = 0;

    while commands.ready() {
        match commands.dequeue() {
            Some(command) => {
                commands_processed += 1;

                use Command::*;
                match command {
                    #[cfg(feature = "master")]
                    GTSStart => {
                        radio.state = send_gts_start(prepare_radio(&mut radio.state), &mut cx, buffer)
                    },
                    DynListen(dyn_slot_duration, radio_config) => { // right now dyn_phase_duration, fire for each slot later
                        cx.tracer.event(TraceEvent::DynWindowStart);
                        let receiving_radio = enable_receiver(prepare_radio(&mut radio.state), radio_config);
                        radio.state = RadioState::DynReceiving((Some(receiving_radio), radio_config));
                    },
                    DynSend(dyn_slot_duration, radio_config) => {
                        cx.tracer.event(TraceEvent::DynWindowStart);
                        radio.state = send_dyn_data(prepare_radio(&mut radio.state), &mut cx, buffer, radio_config);
                    },
                    GTSEnd => {
                        cx.tracer.event(TraceEvent::GTSEnded);
                    },
                    #[cfg(feature = "slave")]
                    ListenForGTSStart(RadioConfig) => {
                        cx.tracer.event(TraceEvent::Listening);
                        let receiving_radio = enable_receiver(prepare_radio(&mut radio.state), RadioConfig::default());
                        radio.state = RadioState::GTSStartWaiting(Some(receiving_radio));
                    },
                    #[cfg(feature = "slave")]
                    SendGTSAnswer(slot_duration, radio_config) => {
                        cx.tracer.event(TraceEvent::GTSAnswerSend);
                        radio.state = send_gts_answer(prepare_radio(&mut radio.state), &mut cx, buffer);
                    },
                    ForceReadyIfSending => {
                        if radio.state.is_sending_state() {
                            radio.state = Ready(Some(prepare_radio(&mut radio.state)));
                        }
                        cx.tracer.event(TraceEvent::ForceReadyIfSending);
                    },
                    ForceReady => {
                        radio.state = Ready(Some(prepare_radio(&mut radio.state)));
                        cx.tracer.event(TraceEvent::ForceReady);
                    },
                    AlohaSlotStart(_, _) => {},
                    RangingStart(_, _) => {}
                };
            }
            _ => {}
        }
    }

    // No need to continue, if at least one command was processed, since no work can be done yet.
    if commands_processed == 1 {
        return;
    } else if commands_processed > 1 {
        rprintln!(=>1, "{}{} commands was processed in a single call, bug?{}\n", color::YELLOW, commands_processed, color::DEFAULT);
        return;
    }

    radio.state = match &mut radio.state {
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            RadioState::Ready(Some(ready_radio))
        },
        #[cfg(feature = "master")]
        GTSStartSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio, &mut cx)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            let answers_received = sd.1;
            advance_gts_answers_receiving(receiving_radio, &mut cx, buffer, answers_received)
        }
        #[cfg(feature = "slave")]
        GTSStartWaiting(sd) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, &mut cx, buffer)
        }
        #[cfg(feature = "slave")]
        GTSAnswerSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_dyn_sending(sending_radio, &mut cx, buffer)
        },
        DynReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_dyn_waiting(receiving_radio, &mut cx, buffer, sd.1)
        },
        DynSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_dyn_sending(sending_radio, &mut cx, buffer)
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
fn send_gts_start<A: Arbiter<Error = Error>, T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8]
) -> RadioState
{
    cx.tracer.event(TraceEvent::GTSStart);

    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
    let mut mux = MiniMultiplexer::new(bufmut);

    Scheduler::source_timeslots(cx.scheduler, &mut mux);
    cx.arbiter.source_sync(&mut mux);

    let bufmut_taken = mux.take();
    len += bufmut_taken.written();

    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let tx_config = get_txconfig(RadioConfig::default());
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
    cx.tracer.event(TraceEvent::GTSStart);
    RadioState::GTSStartSending(Some(sending_radio))
}

#[cfg(feature = "master")]
fn advance_gts_start_sending<A: Arbiter, T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<A, T>
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::TimingMarker);
            let instant = CycntInstant::now();
            cx.spawn.radio_event(Event::TimeMark(instant)).ok();

            // Enable receiver right away
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
            cx.tracer.event(TraceEvent::GTSStartedReceivingAnswers);

            // Schedule gts phase end check.
            let dt: MicroSeconds = Scheduler::gts_period_end();
            cx.schedule.radio_event(
                instant + us2cycles!(cx.clocks, dt.0),
                Event::GTSShouldHaveEnded
            ).ok(); // TODO: count

            // Schedule Aloha period start and end.
            let dt: MicroSeconds = Scheduler::aloha_period_start();
            cx.schedule.radio_event(
                instant + us2cycles!(cx.clocks, dt.0),
                Event::AlohaSlotAboutToStart(Scheduler::aloha_phase_duration(), RadioConfig::default())
            ).ok(); // TODO: count
            let dt: MicroSeconds = Scheduler::aloha_period_end();
            cx.schedule.radio_event(
                instant + us2cycles!(cx.clocks, dt.0),
                Event::AlohaSlotEnded
            ).ok(); // TODO: count

            // Schedule dyn slots start and end.
            let quarter_guard: MicroSeconds = Scheduler::quarter_guard();
            let quarter_guard = us2cycles!(cx.clocks, quarter_guard.0);
            //let mut dyn_uplink_slots = cx.scheduler.into_iter().filter(|s| s.slot_type == SlotType::DynUplink);
            for s in cx.scheduler.into_iter().filter(|s| s.slot_type == SlotType::DynUplink) {
                let shift: MicroSeconds = s.shift;
                let shift = us2cycles!(cx.clocks, shift.0);
                cx.schedule.radio_event(
                    instant + shift,
                    Event::DynUplinkAboutToStart(s.duration, s.radio_config)
                ).ok();
                let duration: MicroSeconds = s.duration;
                let duration = us2cycles!(cx.clocks, duration.0);
                cx.schedule.radio_event(
                    instant + shift + duration + quarter_guard,
                    Event::DynShouldHaveEnded
                ).ok();
            }

            RadioState::GTSAnswersReceiving((Some(receiving_radio), 0))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSStartSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"{}GTSStart send error: {:?}{}", color::RED, e, color::DEFAULT);
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
    mut cx: &mut SMContext<A, T>,
    message: dw1000::hl::Message,
    answers_received: u8,
) -> RadioState
{
    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    rprintln!(=>2,"{}G.RX {}{}\n", color::CYAN, message.frame.payload.len(), color::DEFAULT);
    let payload = message.frame.payload;
    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    demux.demux(config::UWB_ADDR, |channel, chunk| {
        if channel.is_ctrl() {

        } else {
            cx.arbiter.sink_sync(message.frame.header.source, channel, chunk);
        }
    });

    let answers_received = answers_received + 1;
    if answers_received == config::REQUIRED_SLAVE_COUNT { // TODO: replace with the actual number of gt slots given out
        cx.spawn.radio_event(Event::GTSProcessingFinished).ok(); // TODO: count
        RadioState::Ready(Some(ready_radio))
    } else {
        let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
        RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
    }
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving<A: Arbiter<Error = Error>, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
    answers_received: u8,
) -> RadioState
{
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
            rprintln!(=>1,"{}GTSAnswer receive error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received))
            }
        }
    }
}

fn send_dyn_data<A: Arbiter<Error = Error>, T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len..]);
    let mut mux = MiniMultiplexer::new(bufmut);

    cx.arbiter.source_async(&mut mux);

    let pong = Pong{};
    mux.mux(&pong, LogicalDestination::Implicit, ChannelId::ctrl());

    let bufmut = mux.take();
    len += bufmut.written();

    let tx_config = get_txconfig(radio_config);
    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
    RadioState::DynSending(Some(sending_radio))
}

fn advance_dyn_sending<A: Arbiter<Error = Error>, T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::MessageSent);
            cx.spawn.radio_event(Event::DynProcessingFinished).ok(); // TODO: count
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // GTSAnswer is still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::DynSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"{}Dyn send error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

fn process_messages_dyn_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<A, T>,
    message: dw1000::hl::Message,
) -> RadioState
{
    let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    rprintln!(=>2,"{}D.RX{} from {:?}{}", color::CYAN, message.frame.payload.len(), message.frame.header.source, color::DEFAULT);
    let payload = message.frame.payload;
    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    demux.demux(config::UWB_ADDR, |channel, chunk| {
        if channel.is_ctrl() {
            let mut buf = Buf::new(&chunk);
            match Pong::des(&mut buf) {
                Ok(_) => { rprintln!(=>2,"Pong from {:?}", message.frame.header.source); },
                Err(_) => {}
            };
        } else {
            cx.arbiter.sink_async(message.frame.header.source, channel, chunk);
        }
    });
    cx.spawn.ctrl_link_control().ok(); // TODO: HACK
    cx.spawn.radio_event(Event::DynProcessingFinished).ok(); // TODO: only after Pong is received!
    RadioState::Ready(Some(ready_radio))
}

fn advance_dyn_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            process_messages_dyn_waiting(receiving_radio, cx, message)
        },
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    rprintln!(=>1, "{}dyn WB{}\n", color::YELLOW, color::DEFAULT);
                    RadioState::DynReceiving((Some(receiving_radio), radio_config))
                },
                nb::Error::Other(e) => {
                    if let dw1000::Error::SfdTimeout = e {

                    } else {
                        rprintln!(=>1, "{}dyn_uw err: {:?}{}\n", color::YELLOW, e, color::DEFAULT);
                    }
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    // Switch back to GTSStart waiting state
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::DynReceiving((Some(receiving_radio), radio_config))
                }
            }
        }
    }
}

#[cfg(feature = "slave")]
fn send_gts_answer<A: Arbiter<Error = Error>, T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
) -> RadioState
{
    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
    let mut mux = MiniMultiplexer::new(bufmut);

    cx.arbiter.source_sync(&mut mux);

    let bufmut_taken = mux.take();
    len += bufmut_taken.written();

    rprintln!(=>5, "{}sourced:{}{}\n\n", color::CYAN, bufmut_taken.written(), color::DEFAULT);

    let tx_config = get_txconfig(RadioConfig::default());

    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], None, tx_config).expect("DW1000 internal failure?");
    RadioState::GTSAnswerSending(Some(sending_radio))
}

#[cfg(feature = "slave")]
fn process_messages_gts_start_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<A, T>,
    message: dw1000::hl::Message,
) -> RadioState
{
    let cycnt_now = CycntInstant::now();
    let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    rprintln!(=>2,"{}G.RX {}{}", color::CYAN, message.frame.payload.len(), /*message.frame.header.source,*/ color::DEFAULT);
    let payload = message.frame.payload;
    // rprint!(=>1, "p:{}[", payload.len());
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
            //rprintln!(=>1, "ctrl_ch:{}\n", chunk.len());
            let mut buf = Buf::new(&chunk);
            match Slot::des(&mut buf) {
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
                Err(e) => {
                    rprintln!(=>1, "{}demux err:{:?}{}", color::YELLOW, e, color::DEFAULT);
                }
            }
        } else {
            cx.arbiter.sink_sync(message.frame.header.source, channel, chunk);
        }
    });

    if is_gts_start {
        cx.tracer.event(TraceEvent::GTSStartReceived);

        // TODO: rx_time is 0 for some reason, need to figure out or ranging would not work
        // let processing_delay = ready_radio.sys_time().unwrap_or(message.rx_time).duration_since(message.rx_time);
        // rprintln!(=>13, "Mrt {}", message.rx_time.value() / 64);
        // rprintln!(=>13, "Pde {}", processing_delay.to_nanos());
        // let processing_delay = (cx.clocks.sysclk().0 / 1_000_000) * (processing_delay.to_nanos() / 1_000) as u32;
        let instant = CycntInstant::now();
        let processing_delay = us2cycles!(cx.clocks, 400);
        cx.spawn.radio_event(Event::GTSStartReceived(instant - processing_delay)).ok(); // TODO: count
    }

    RadioState::Ready(Some(ready_radio))
}

#[cfg(feature = "slave")]
fn advance_gts_start_waiting<A: Arbiter<Error = Error>, T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
) -> RadioState
{
    //let sys_status = receiving_radio.ll().sys_status().read().unwrap();
    //rprintln!(=>1,"A: {}", sys_status);

    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            *cx.state_instant = None;
            cx.tracer.event(TraceEvent::MessageReceived);
            process_messages_gts_start_waiting(receiving_radio, cx, message)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                if cx.state_instant.is_none() {
                    *cx.state_instant = Some(CycntInstant::now());
                } else {
                    let elapsed = cx.state_instant.unwrap().elapsed().as_cycles();
                    if elapsed > 172_000_000 {
                        *cx.state_instant = Some(CycntInstant::now());
                        rprintln!(=>1,"ReListen");
                        panic!("ReListen");
                        let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure");
                        let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                        return RadioState::GTSStartWaiting(Some(receiving_radio));
                    }
                }
                RadioState::GTSStartWaiting(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!(=>1,"{}RX error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::GTSStartWaiting(Some(receiving_radio))
            }
        }
    }
}

#[cfg(feature = "slave")]
fn advance_gts_answer_sending<A: Arbiter<Error = Error>, T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<A, T>,
    buffer: &mut[u8],
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::GTSAnswerSent);
            cx.spawn.radio_event(Event::GTSProcessingFinished).ok(); // TODO: count
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // GTSAnswer is still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSAnswerSending(Some(sending_radio))
            } else { // Actuall error while sending
                rprintln!(=>1,"{}GTS send error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}
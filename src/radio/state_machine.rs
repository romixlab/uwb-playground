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
use dw1000::time::{Instant as RadioInstant, Instant};
use dw1000::time::Duration as RadioDuration;
use cfg_if::cfg_if;
use embedded_hal::digital::v2::OutputPin;
use crate::config;
use crate::color;
use crate::config::Dw1000Cs;
use crate::util::{Tracer, TraceEvent};
use dw1000::hl::SendTime;
use rtic::Mutex;

const SM_FAIL_MESSAGE: &'static str = "Radio state machine fail";

/// Convenient way to pass around all the fields below.
struct SMContext<'a, 'c, T> {
    channels: &'a mut crate::resources::channels<'c>,
    tracer: &'a mut T,
    spawn: &'a crate::radio_irq::Spawn<'a>,
    schedule: &'a crate::radio_irq::Schedule<'a>,
    clocks: &'a crate::board::hal::rcc::Clocks,
    scheduler: &'a mut Scheduler,
    #[cfg(any(feature = "slave", feature = "anchor"))]
    master_node: &'a mut NodeState,
    state_instant: &'a mut Option<CycntInstant>,
    watchdog: &'a mut IndependentWatchdog,
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
        GTSStartSending((rs, _)) => {
            let sending_radio = rs.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(feature = "master")]
        GTSAnswersReceiving(rs) => {
            let receiving_radio = rs.0.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSStartWaiting((rs, _)) => {
            let receiving_radio = rs.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
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
        RangingPingSending(rs) |
        RangingRequestSending(rs) |
        RangingResponseSending(rs) => {
            let sending_radio = rs.0.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        }
        RangingPingWaiting(rs) |
        RangingRequestWaiting(rs) |
        RangingResponseWaiting(rs) => {
            let receiving_radio = rs.0.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        }
        OneOffSending(rs) => {
            let sending_radio = rs.take().expect(SM_FAIL_MESSAGE);
            sending_radio.finish_sending().expect(SM_FAIL_MESSAGE)
        },
        Listening(rs) => {
            let receiving_radio = rs.take().expect(SM_FAIL_MESSAGE);
            receiving_radio.finish_receiving().expect(SM_FAIL_MESSAGE)
        }
    };
    ready_radio
}


pub fn advance<'a, 'b, 'c, T: Tracer>(
    radio: &'a mut Radio,
    channels: &'a mut crate::resources::channels<'c>,
    buffer: &'b mut[u8],
    spawn: &'a crate::radio_irq::Spawn,
    schedule: &'a crate::radio_irq::Schedule,
    clocks: &'a crate::board::hal::rcc::Clocks,
    scheduler: &'a mut Scheduler,
    watchdog: &'a mut IndependentWatchdog,
    tracer: &'a mut T
) {
    use RadioState::*;
    // Hint: do not try to put buffer into the SMContext struct, borrow checker would not be happy
    cfg_if! {
        if #[cfg(feature = "master")] {
            let mut cx = SMContext { channels, tracer, spawn, schedule, clocks, scheduler, state_instant: &mut radio.state_instant, watchdog };
        } else if #[cfg(any(feature = "slave", feature = "anchor"))] {
            let mut cx = SMContext { channels, tracer, spawn, schedule, clocks, scheduler, master_node: &mut radio.master, state_instant: &mut radio.state_instant, watchdog };
        }
    }

    let commands = &mut radio.commands;
    let mut commands_processed = 0;

    // rprint!(=>13, "I-{:?} -> ", radio.state);
    while commands.ready() {
        match commands.dequeue() {
            Some(command) => {
                commands_processed += 1;

                use Command::*;
                match command {
                    #[cfg(feature = "master")]
                    GTSStart(radio_config) => {
                        radio.state = send_gts_start(prepare_radio(&mut radio.state), &mut cx, buffer, radio_config)
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
                    #[cfg(any(feature = "slave", feature = "anchor"))]
                    ListenForGTSStart(radio_config) => {
                        cx.tracer.event(TraceEvent::Listening);
                        let receiving_radio = enable_receiver(prepare_radio(&mut radio.state), radio_config);
                        radio.state = RadioState::GTSStartWaiting((Some(receiving_radio), radio_config));
                    },
                    #[cfg(any(feature = "slave", feature = "anchor"))]
                    SendGTSAnswer(slot_duration, radio_config) => {
                        cx.tracer.event(TraceEvent::GTSAnswerSend);
                        radio.state = send_gts_answer(prepare_radio(&mut radio.state), &mut cx, buffer, radio_config);
                    },
                    ForceReadyIfSending => {
                        cx.tracer.event(TraceEvent::ForceReadyIfSending);
                        if radio.state.is_sending_state() {
                            radio.state = Ready(Some(prepare_radio(&mut radio.state)));
                        }
                    },
                    ForceReady => {
                        cx.tracer.event(TraceEvent::ForceReady);
                        radio.state = Ready(Some(prepare_radio(&mut radio.state)));
                    },
                    AlohaSlotStart(_, _) => {},
                    RangingStart(slot_duration, radio_config) => {
                        cx.tracer.event(TraceEvent::RangingStarted);
                        let now = CycntInstant::now();
                        radio.state = advance_ranging_ready(prepare_radio(&mut radio.state), &mut cx, buffer, now, slot_duration, radio_config);
                    }
                    SetAntennaDelay(tx_delay, rx_delay) => {
                        let mut ready_radio = prepare_radio(&mut radio.state);
                        let _ = ready_radio.set_antenna_delay(rx_delay, tx_delay);
                        radio.state = RadioState::Ready(Some(ready_radio));
                    }
                    SendMessage(radio_config, address, dummy) => {
                        let mut ready_radio = prepare_radio(&mut radio.state);
                        let sending_radio = send_message_no_mux(
                            ready_radio,
                            dummy,
                            address,
                            buffer,
                            radio_config,
                            SendTime::Now
                        );
                        radio.state = RadioState::OneOffSending(Some(sending_radio));
                    },
                    Listen(radio_config) => {
                        let mut ready_radio = prepare_radio(&mut radio.state);
                        let receiving_radio = enable_receiver(ready_radio, radio_config);
                        radio.state = RadioState::Listening(Some(receiving_radio));
                    }
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

    // rprint!(=>13, "C-{:?} -> ", radio.state);
    radio.state = match &mut radio.state {
        Ready(sd) => {
            let ready_radio = sd.take().expect(SM_FAIL_MESSAGE);
            RadioState::Ready(Some(ready_radio))
        },
        #[cfg(feature = "master")]
        GTSStartSending((sd, radio_config)) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_sending(sending_radio, &mut cx, *radio_config)
        }
        #[cfg(feature = "master")]
        GTSAnswersReceiving((sd, answers_received, radio_config)) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answers_receiving(receiving_radio, &mut cx, buffer, *answers_received, *radio_config)
        }
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSStartWaiting((sd, radio_config)) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_start_waiting(receiving_radio, &mut cx, buffer, *radio_config)
        }
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSAnswerSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_gts_answer_sending(sending_radio, &mut cx, buffer)
        },
        DynReceiving(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_dyn_waiting(receiving_radio, &mut cx, buffer, sd.1)
        },
        DynSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_dyn_sending(sending_radio, &mut cx, buffer)
        }
        RangingPingSending(sd) => {
            let sending_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_ping_sending(sending_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        RangingPingWaiting(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_ping_receiving(receiving_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        RangingRequestSending(sd) => {
            let sending_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_request_sending(sending_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        RangingRequestWaiting(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_request_receiving(receiving_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        RangingResponseSending(sd) => {
            let sending_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_response_sending(sending_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        RangingResponseWaiting(sd) => {
            let receiving_radio = sd.0.take().expect(SM_FAIL_MESSAGE);
            advance_ranging_response_receiving(receiving_radio, &mut cx, buffer, sd.1, sd.2, sd.3)
        }
        OneOffSending(sd) => {
            let sending_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_sending_to_ready(sending_radio, &mut cx)
        },
        Listening(sd) => {
            let receiving_radio = sd.take().expect(SM_FAIL_MESSAGE);
            advance_listening_to_listening(receiving_radio, &mut cx, buffer)
        }
    };
    // rprintln!(=>13, "E-{:?}", radio.state);
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
        frame_filtering: true,
        pulse_repetition_frequency: radio_config.prf,
    };
    ready_radio.enable_rx_interrupts().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    ready_radio.receive(rx_config).expect("dw1000 crate or spi failure") // TODO: try to re-init and recover
}

pub fn default_mac_frame(payload: &[u8]) -> mac::Frame {
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
fn send_gts_start<T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    cx.tracer.event(TraceEvent::GTSStart);

    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
    let mut mux = MiniMultiplexer::new(bufmut);

    Scheduler::source_timeslots(cx.scheduler, &mut mux);
    cx.channels.lock(|channels: &mut crate::channels::Channels| channels.source_sync(&mut mux) );

    let bufmut_taken = mux.take();
    len += bufmut_taken.written();

    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let tx_config = get_txconfig(radio_config);
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], SendTime::Now, tx_config).expect("DW1000 internal failure?");
    cx.tracer.event(TraceEvent::GTSStart);
    RadioState::GTSStartSending((Some(sending_radio), radio_config))
}

#[cfg(feature = "master")]
fn advance_gts_start_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
    radio_config: RadioConfig
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::TimingMarker);
            let instant = CycntInstant::now();
            cx.spawn.radio_event(Event::TimeMark(instant)).ok();

            // Enable receiver right away
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, radio_config);
            cx.tracer.event(TraceEvent::GTSStartedReceivingAnswers);

            // Schedule gts phase end check.
            let dt: MicroSeconds = Scheduler::gts_period_end();
            cx.schedule.radio_event(
                instant + us2cycles!(cx.clocks, dt.0),
                Event::GTSShouldHaveEnded
            ).ok(); // TODO: count

            // Schedule Aloha slot start and end.
            // let dt: MicroSeconds = Scheduler::aloha_period_start();
            // cx.schedule.radio_event(
            //     instant + us2cycles!(cx.clocks, dt.0),
            //     Event::AlohaSlotAboutToStart(Scheduler::aloha_phase_duration(), RadioConfig::default())
            // ).ok(); // TODO: count
            // let dt: MicroSeconds = Scheduler::aloha_period_end();
            // cx.schedule.radio_event(
            //     instant + us2cycles!(cx.clocks, dt.0),
            //     Event::AlohaSlotEnded
            // ).ok(); // TODO: count

            // Schedule dyn slots start and end.
            // let quarter_guard: MicroSeconds = Scheduler::quarter_guard();
            // let quarter_guard = us2cycles!(cx.clocks, quarter_guard.0);
            // //let mut dyn_uplink_slots = cx.scheduler.into_iter().filter(|s| s.slot_type == SlotType::DynUplink);
            // for s in cx.scheduler.into_iter().filter(|s| s.slot_type == SlotType::DynUplink) {
            //     let shift: MicroSeconds = s.shift;
            //     let shift = us2cycles!(cx.clocks, shift.0);
            //     cx.schedule.radio_event(
            //         instant + shift,
            //         Event::DynUplinkAboutToStart(s.duration, s.radio_config)
            //     ).ok();
            //     let duration: MicroSeconds = s.duration;
            //     let duration = us2cycles!(cx.clocks, duration.0);
            //     cx.schedule.radio_event(
            //         instant + shift + duration + quarter_guard,
            //         Event::DynShouldHaveEnded
            //     ).ok();
            // }

            // Schedule ranging slot start and end.
            // let dt: MicroSeconds = Scheduler::ranging_period_start();
            // cx.schedule.radio_event(
            //     instant + us2cycles!(cx.clocks, dt.0),
            //     Event::RangingSlotAboutToStart(Scheduler::ranging_phase_duration(), RadioConfig::fast())
            // ).ok(); // TODO: count
            // let dt: MicroSeconds = Scheduler::ranging_period_end();
            // cx.schedule.radio_event(
            //     instant + us2cycles!(cx.clocks, dt.0),
            //     Event::RangingSlotEnded
            // ).ok(); // TODO: count

            RadioState::GTSAnswersReceiving((Some(receiving_radio), 0, radio_config))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still sending
                //rprintln!(=>1,"blockon:{:?}", e);
                RadioState::GTSStartSending((Some(sending_radio), radio_config))
            } else { // Actuall error while sending
                rprintln!(=>1,"{}GTSStart send error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                //tracer.event(TraceEvent::XYZ);
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

fn advance_sending_to_ready<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            cx.tracer.event(TraceEvent::MessageSent);
            RadioState::Ready(Some(ready_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e {
                RadioState::OneOffSending(Some(sending_radio))
            } else {
            rprintln!(=>1,"{}One off send error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                RadioState::Ready(Some(ready_radio))
            }
        }
    }
}

fn advance_listening_to_listening<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok(message) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            cx.watchdog.feed();
            //rprintln!(=>10, "{:?}\n", receiving_radio.calculate_rssi());
            let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
            RadioState::Listening(Some(receiving_radio))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e {
                RadioState::Listening(Some(receiving_radio))
            } else { // Actuall error while receiving
                rprintln!(=>1,"{}L re:{:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                let receiving_radio = enable_receiver(ready_radio, RadioConfig::default());
                RadioState::Listening(Some(receiving_radio))
            }
        }
    }
}

#[cfg(feature = "master")]
fn process_messages_gts_answers_receiving<T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    message: dw1000::hl::Message,
    answers_received: u8,
    radio_config: RadioConfig
) -> RadioState
{
    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover

    let radio_state = if answers_received == config::REQUIRED_SLAVE_COUNT { // TODO: replace with the actual number of gt slots given out
        cx.spawn.radio_event(Event::GTSProcessingFinished).ok(); // TODO: count
        RadioState::Ready(Some(ready_radio))
    } else {
        let receiving_radio = enable_receiver(ready_radio, radio_config);
        RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received, radio_config))
    };

    //rprintln!(=>2,"{}G.RX {}{}\n", color::CYAN, message.frame.payload.len(), color::DEFAULT);
    let payload = message.frame.payload;
    let mut buf = Buf::new(payload);
    let mut demux = MiniDemultiplexer::new(&mut buf);
    let mut demuxed = 0;
    demux.demux(config::UWB_ADDR, |channel, chunk| {
        if channel.is_ctrl() {

        } else {
            cx.channels.lock(|channels: &mut crate::channels::Channels| channels.sink_sync(message.frame.header.source, channel, chunk) );
            demuxed += 1;
        }
    });
   //rprintln!(=>2, "DEMUXED: {}", demuxed);

    let answers_received = answers_received + 1;
    radio_state
}

#[cfg(feature = "master")]
fn advance_gts_answers_receiving<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    answers_received: u8,
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok(message) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            cx.watchdog.feed();
            cx.spawn.radio_event(Event::MessageReceived(message.0.frame.header.source, message.0.frame.payload.len()));
            //rprintln!(=>10, "{:?}\n", receiving_radio.calculate_rssi());
            process_messages_gts_answers_receiving(receiving_radio, cx, message.0, answers_received, radio_config)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                //rprintln!(=>1,"blockon:{:?}", e);
                //let sys_status = receiving_radio.ll().sys_status().read();
                //rprintln!(=>1,"sstY {:?}", sys_status);
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received, radio_config))
            } else { // Actuall error while receiving
            rprintln!(=>1,"{}GTSAnswer receive error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, radio_config);
                RadioState::GTSAnswersReceiving((Some(receiving_radio), answers_received, radio_config))
            }
        }
    }
}

fn send_dyn_data<T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len..]);
    let mut mux = MiniMultiplexer::new(bufmut);

    cx.channels.lock(|channels: &mut crate::channels::Channels| channels.source_async(&mut mux));

    let pong = Pong{};
    let _ = mux.mux(&pong, LogicalDestination::Implicit, ChannelId::ctrl());

    let bufmut = mux.take();
    len += bufmut.written();

    let tx_config = get_txconfig(radio_config);
    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], SendTime::Now, tx_config).expect("DW1000 internal failure?");
    RadioState::DynSending(Some(sending_radio))
}

fn advance_dyn_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
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

fn process_messages_dyn_waiting<T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
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
            cx.channels.lock(|channels: &mut crate::channels::Channels| channels.sink_async(message.frame.header.source, channel, chunk));
        }
    });
    cx.spawn.radio_event(Event::DynProcessingFinished).ok(); // TODO: only after Pong is received!
    RadioState::Ready(Some(ready_radio))
}

fn advance_dyn_waiting<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            cx.watchdog.feed();
            //rprintln!(=>10, "{:?}\n", receiving_radio.calculate_rssi());
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

#[cfg(any(feature = "slave", feature = "anchor"))]
fn send_gts_answer<T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    let frame = default_mac_frame(&[]);
    let mut len = frame.encode(buffer, mac::WriteFooter::No);

    let mut bufmut = BufMut::new(&mut buffer[len .. len + 1000]);
    let mut mux = MiniMultiplexer::new(bufmut);

    cx.channels.lock(|channels: &mut crate::channels::Channels| channels.source_sync(&mut mux));

    let bufmut_taken = mux.take();
    len += bufmut_taken.written();

    // rprintln!(=>5, "{}sourced:{}{}\n\n", color::CYAN, bufmut_taken.written(), color::DEFAULT);

    let tx_config = get_txconfig(radio_config);

    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    let mut sending_radio = ready_radio.send_raw(&buffer[0..len], SendTime::Now, tx_config).expect("DW1000 internal failure?");
    RadioState::GTSAnswerSending(Some(sending_radio))
}

#[cfg(any(feature = "slave", feature = "anchor"))]
fn process_messages_gts_start_waiting<T: Tracer>(
    receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    message: dw1000::hl::Message,
    radio_config: RadioConfig
) -> RadioState
{
    let cycnt_now = CycntInstant::now();
    let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
    // rprintln!(=>2,"{}G.RX {}{}", color::CYAN, message.frame.payload.len(), /*message.frame.header.source,*/ color::DEFAULT);
    let payload = message.frame.payload;
    // rprint!(=>1, "p:{}[", payload.len());
    // for b in payload {
    //     rprint!(=>1, "{:02x} ", b);
    // }
    // rprintln!(=>1, "]\n");

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
            cx.channels.lock(|channels: &mut crate::channels::Channels| channels.sink_sync(message.frame.header.source, channel, chunk));
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
        RadioState::Ready(Some(ready_radio))
    } else {
        let receiving_radio = enable_receiver(ready_radio, radio_config);
        RadioState::GTSStartWaiting((Some(receiving_radio), radio_config))
    }
}

#[cfg(any(feature = "slave", feature = "anchor"))]
fn advance_gts_start_waiting<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    radio_config: RadioConfig
) -> RadioState
{
    //let sys_status = receiving_radio.ll().sys_status().read().unwrap();
    //rprintln!(=>1,"A: {}", sys_status);

    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            *cx.state_instant = None;
            cx.tracer.event(TraceEvent::MessageReceived);
            cx.watchdog.feed();
            //rprintln!(=>10, "{:?}\n", receiving_radio.calculate_rssi());
            process_messages_gts_start_waiting(receiving_radio, cx, message, radio_config)
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // Still receiving
                if cx.state_instant.is_none() {
                    *cx.state_instant = Some(CycntInstant::now());
                } else {
                    let elapsed = cx.state_instant.unwrap().elapsed().as_cycles();
                    if elapsed > 172_000_000 {
                        *cx.state_instant = Some(CycntInstant::now());
                        // rprintln!(=>1,"ReListen");
                        //panic!("ReListen");
                        // let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure");
                        // let receiving_radio = enable_receiver(ready_radio, radio_config);
                        return RadioState::GTSStartWaiting((Some(receiving_radio), radio_config));
                    }
                }
                RadioState::GTSStartWaiting((Some(receiving_radio), radio_config))
            } else { // Actuall error while receiving
                rprintln!(=>1,"{}RX error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                // Switch back to GTSStart waiting state
                let receiving_radio = enable_receiver(ready_radio, radio_config);
                RadioState::GTSStartWaiting((Some(receiving_radio), radio_config))
            }
        }
    }
}

#[cfg(any(feature = "slave", feature = "anchor"))]
fn advance_gts_answer_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
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

use dw1000::ranging::{Ping as RangingPing, Request as RangingRequest, Response as RangingResponse, RxMessage};
use crate::radio::serdes_impl;
use dw1000::mac::Address;
use no_std_compat::marker::PhantomData;
use stm32g4xx_hal::watchdog::IndependentWatchdog;
use embedded_hal::watchdog::Watchdog;

pub fn send_message_no_mux<S: crate::radio::serdes::Serialize>(
    mut ready_radio: ReadyRadio,
    message: S,
    destination: Address,
    buffer: &mut[u8],
    radio_config: RadioConfig,
    send_time: SendTime,
) -> SendingRadio {
    let mut frame = default_mac_frame(&[]);
    frame.header.destination = destination;
    let mut len = frame.encode(buffer, mac::WriteFooter::No);
    let mut bufmut = BufMut::new(&mut buffer[len .. len + 128]);
    let _ = message.ser(&mut bufmut);
    len += bufmut.written();

    ready_radio.enable_tx_interrupts().ok(); // TODO: count errors
    ready_radio.send_raw(&buffer[0..len], send_time, radio_config.into()).expect("DW1000 internal failure?")
}

const RANGING_PROCESSING_DURATION_NS: u32 = 700_000_u32; // TODO: timing trainer!
fn advance_ranging_ready<T: Tracer>(
    mut ready_radio: ReadyRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    cfg_if! {
        if #[cfg(feature = "anchor")] {
            let ping = RangingPing::new(&mut ready_radio, RadioDuration::from_nanos(8_000_000)).expect("Ping::new()");
            rprintln!(=>6, "\n----------\nPing to send: {:?}\r", ping);
            let sending_radio = send_message_no_mux(ready_radio, ping.payload, Address::broadcast(&mac::AddressMode::Short), buffer, radio_config, SendTime::Delayed(ping.tx_time));
            RadioState::RangingPingSending((Some(sending_radio), slot_started, slot_duration, radio_config))
        } else {
            let receiving_radio = enable_receiver(ready_radio, radio_config);
            RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
        }
    }

}

fn advance_ranging_ping_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::MessageSent);
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, radio_config);
            RadioState::RangingRequestWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e {
                RadioState::RangingPingSending((Some(sending_radio), slot_started, slot_duration, radio_config))
            } else { // Actuall error while sending
                rprintln!(=>6,"{}Ping send err: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                let receiving_radio = enable_receiver(ready_radio, radio_config);
                RadioState::RangingRequestWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
            }
        }
    }
}

fn advance_ranging_ping_receiving<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            cx.tracer.event(TraceEvent::MessageReceived);

            let payload = message.frame.payload;
            let mut buf = Buf::new(payload);
            let ping = RangingPing::des(&mut buf);
            match ping {
                Ok(ping) => {
                    let ping = RxMessage {
                        rx_time: message.rx_time,
                        source: message.frame.header.source,
                        payload: ping
                    };
                    //rprintln!(=>6, "\n-------\nPing rx_time (local)[ticks] {}\n", ping.rx_time.value());
                    //rprintln!(=>6, "ping tx_time (received)[ticks]: {}\n", ping.payload.ping_tx_time.value());
                    let ranging_request = RangingRequest::new(
                        &mut ready_radio,
                        &ping,
                        RadioDuration::from_nanos(400_000)
                    ).expect("RangingRequest::new()");
                    //rprintln!(=>6, "request: ping_reply[ns]: {}\n", ranging_request.payload.ping_reply_time.to_nanos());
                    //rprintln!(=>6, "request: req_tx[ticks]: {}\n", ranging_request.payload.ping_tx_time.value());

                    let sending_radio = send_message_no_mux(ready_radio, ranging_request.payload, ping.source, buffer, radio_config, SendTime::Delayed(ranging_request.tx_time));
                    RadioState::RangingRequestSending((Some(sending_radio), slot_started, slot_duration, radio_config))
                },
                Err(_) => {
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        },
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                },
                nb::Error::Other(e) => {
                    if let dw1000::Error::SfdTimeout = e {

                    } else {
                        rprintln!(=>6, "{}Ping rx err: {:?}{}\n", color::YELLOW, e, color::DEFAULT);
                    }
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        }
    }
}

fn advance_ranging_request_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::MessageSent);
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, radio_config);
            RadioState::RangingResponseWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e {
                RadioState::RangingRequestSending((Some(sending_radio), slot_started, slot_duration, radio_config))
            } else { // Actuall error while sending
                rprintln!(=>6,"{}Request tx err: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                let receiving_radio = enable_receiver(ready_radio, radio_config);
                RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
            }
        }
    }
}

fn advance_ranging_request_receiving<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            cx.tracer.event(TraceEvent::MessageReceived);

            let payload = message.frame.payload;
            let mut buf = Buf::new(payload);
            let ranging_request = RangingRequest::des(&mut buf);
            match ranging_request {
                Ok(ranging_request) => {
                    let ranging_request = RxMessage {
                        rx_time: message.rx_time,
                        source: message.frame.header.source,
                        payload: ranging_request
                    };
                    rprintln!(=>6, "request: ping_reply[ticks]: {}\n", ranging_request.payload.ping_tx_time.value());
                    //rprintln!(=>6, "request: ping_reply[ns]: {}\n", ranging_request.payload.ping_reply_time.to_nanos());
                    //rprintln!(=>6, "request: req_tx[ticks]: {}\n", ranging_request.payload.ping_tx_time.value());
                    let ranging_response = RangingResponse::new(
                        &mut ready_radio,
                        &ranging_request,
                        RadioDuration::from_nanos(400_000)
                    ).expect("RangingResponse::new()");
                    //rprintln!(=>6, "response: ping_reply_time[ns]: {}\n", ranging_response.payload.ping_reply_time.to_nanos());
                    //rprintln!(=>6, "response: ping_reply_time[ticks]: {}\n", ranging_response.payload.request_tx_time.value());
                    //rprintln!(=>6, "response: ping_round_trip_time[ns]: {}\n", ranging_response.payload.ping_round_trip_time.to_nanos());
                    //rprintln!(=>6, "response: request_reply_time[ns]: {}\n", ranging_response.payload.request_reply_time.to_nanos());

                    let sending_radio = send_message_no_mux(ready_radio, ranging_response.payload, ranging_request.source, buffer, radio_config, SendTime::Delayed(ranging_response.tx_time));
                    RadioState::RangingResponseSending((Some(sending_radio), slot_started, slot_duration, radio_config))
                },
                Err(_) => {
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        },
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                },
                nb::Error::Other(e) => {
                    if let dw1000::Error::SfdTimeout = e {

                    } else {
                        rprintln!(=>6, "{}R.Request rx err: {:?}{}\n", color::YELLOW, e, color::DEFAULT);
                    }
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        }
    }
}

fn advance_ranging_response_sending<T: Tracer>(
    mut sending_radio: SendingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match sending_radio.wait() {
        Ok(_) => {
            cx.tracer.event(TraceEvent::MessageSent);
            let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let receiving_radio = enable_receiver(ready_radio, radio_config);
            RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
        },
        Err(e) => {
            if let nb::Error::WouldBlock = e { // GTSAnswer is still sending
                RadioState::RangingResponseSending((Some(sending_radio), slot_started, slot_duration, radio_config))
            } else { // Actuall error while sending
            rprintln!(=>6,"{}R.Response tx error: {:?}{}", color::RED, e, color::DEFAULT);
                let ready_radio = sending_radio.finish_sending().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                let receiving_radio = enable_receiver(ready_radio, radio_config);
                RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
            }
        }
    }
}

fn advance_ranging_response_receiving<T: Tracer>(
    mut receiving_radio: ReceivingRadio,
    mut cx: &mut SMContext<T>,
    buffer: &mut[u8],
    slot_started: CycntInstant,
    slot_duration: MicroSeconds,
    radio_config: RadioConfig
) -> RadioState
{
    match receiving_radio.wait(buffer) {
        Ok((message, _sys_status_before)) => {
            cx.tracer.event(TraceEvent::MessageReceived);
            let quality = receiving_radio.read_rx_quality();
            //rprintln!(=>6, "Quality: {:?}", quality);
            let mut ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
            let (rssi, los) = match quality {
                Ok(quality) => {
                    (quality.rssi, quality.los_confidence_level)
                },
                Err(_) => (-1.0, -1.0)
            };


            let payload = message.frame.payload;
            let mut buf = Buf::new(payload);
            let ranging_response = RangingResponse::des(&mut buf);
            match ranging_response {
                Ok(ranging_response) => {
                    let ranging_response = RxMessage {
                        rx_time: message.rx_time,
                        source: message.frame.header.source,
                        payload: ranging_response
                    };
                    //rprintln!(=>6, "response: ping_reply_time[ns]: {}\n", ranging_response.payload.ping_reply_time.to_nanos());
                    //rprintln!(=>6, "response: ping_reply_time[ticks]: {}\n", ranging_response.payload.request_tx_time.value());
                    //rprintln!(=>6, "response: ping_round_trip_time[ns]: {}\n", ranging_response.payload.ping_round_trip_time.to_nanos());
                    //rprintln!(=>6, "response: request_reply_time[ns]: {}\n", ranging_response.payload.request_reply_time.to_nanos());
                    let ping_rt = ranging_response.payload.ping_reply_time.value();
                    let ping_rtt = ranging_response.payload.ping_round_trip_time.value();
                    let request_rt = ranging_response.payload.request_reply_time.value();
                    let request_rtt = ranging_response.rx_time
                        .duration_since(ranging_response.payload.request_tx_time)
                        .value();

                    let distance = dw1000::ranging::compute_distance_mm(&ranging_response);
                    static mut DIST_COUNTER: u32 = 0u32;
                    unsafe { DIST_COUNTER += 1; }
                    match distance {
                        Ok(d) => {
                            rprintln!(=>6, "{},{}.{},{},{},{},{},{:.2},{:.2}\n",
                                unsafe { DIST_COUNTER },
                                d / 1000, d % 1000,
                                ping_rtt,
                                request_rtt,
                                ping_rt,
                                request_rt,
                                rssi,
                                los
                                );
                        },
                        Err(e) => {
                            rprintln!(=>6, "D err: {:?}\n", e);
                        }
                    }

                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                },
                Err(_) => {
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        },
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                },
                nb::Error::Other(e) => {
                    if let dw1000::Error::SfdTimeout = e {

                    } else {
                        rprintln!(=>6, "{}R.Response rx err: {:?}{}\n", color::YELLOW, e, color::DEFAULT);
                    }
                    let ready_radio = receiving_radio.finish_receiving().expect("dw1000 crate or spi failure"); // TODO: try to re-init and recover
                    let receiving_radio = enable_receiver(ready_radio, radio_config);
                    RadioState::RangingPingWaiting((Some(receiving_radio), slot_started, slot_duration, radio_config))
                }
            }
        }
    }
}
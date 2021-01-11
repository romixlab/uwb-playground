use crate::config;
use crate::radio;
use crate::board::hal;
use crate::radio::types::{Command, Event, RadioConfig, SlotType};
use rtic::Mutex;
use rtt_target::{ rprint, rprintln };
use rtic::cyccnt::U32Ext;
#[cfg(feature = "pozyx-board")]
use hal::gpio::ExtiPin;
use embedded_hal::digital::v2::{OutputPin, InputPin, ToggleableOutputPin};
use crate::util::{Tracer, TraceEvent};
use cfg_if::cfg_if;
use crate::radio::types::{
    Node,
    NodeState
};
use crate::units::{MicroSeconds, U32UnitsExt, us, MilliSeconds};
use crate::color;
use crate::radio::scheduler::Scheduler;
use rtic::cyccnt::Instant as CycntInstant;

pub struct RttTracer {
    pub prev: u32,
    pub prev_gts: u32,
    pub sysclk: u32
}

impl Tracer for RttTracer {
    fn event(&mut self, e: TraceEvent) {
        use cortex_m::peripheral::DWT;

        let now = DWT::get_cycle_count();

        use crate::board::hal::stm32::Peripherals;
        let device = unsafe { Peripherals::steal() };
        let count = e as u8;

        cfg_if!{
            if #[cfg(feature = "gcarrier-board")] {
                let gpioc = device.GPIOC;
                for _ in 0..count {
                    gpioc.bsrr.write(|w| w.bs3().set_bit()); // PC3
                    cortex_m::asm::nop();
                    gpioc.bsrr.write(|w| w.br3().set_bit());
                }
            }
        }

        cfg_if! {
            if #[cfg(feature = "event-rtt-trace")] {
                let dt_gts = now - self.prev_gts;
                let dt_prev = now - self.prev;
                if e == TraceEvent::GTSStart {
                    rprintln!(=> 13, "\n\x1b[2J\x1b[0m---\n");
                } else if e == TraceEvent::TimingMarker {
                    self.prev_gts = now;
                } else if e == TraceEvent::GTSStartReceived {
                    // Cannot easily forward this number for now, see https://github.com/rust-lang/rust/issues/60553
                    let rx_to_process_delay = 155u32; // [us] from rx timestamp till this moment
                    self.prev_gts = now - self.sysclk / 1_000_000 * rx_to_process_delay;
                    rprintln!(=> 13, "\n\n\n---\n");
                }
                self.prev = now;

                let dt_gts = cycles2us_alt!(self.sysclk, dt_gts);
                let dt_prev = cycles2us_alt!(self.sysclk, dt_prev);
                rprintln!(=> 13,
                    "{}({}) +{}.{:03} d{}.{:03}\n",
                    e,
                    count,
                    dt_gts / 1000, dt_gts % 1000,
                    dt_prev / 1000, dt_prev % 1000
                );
            }
        }
    }
}

pub struct NoOpTracer {}

impl Tracer for NoOpTracer {
    fn event(&mut self, _e: TraceEvent) { }
}

#[cfg(feature = "gcarrier-board")]
static mut TRACER: RttTracer = RttTracer { prev: 0, prev_gts: 0, sysclk: 160_000_000 };

pub fn radio_irq(mut cx: crate::radio_irq::Context, buffer: &mut[u8], ) {
    // let now = DWT::get_cycle_count() as i32;
    // let dt = now.wrapping_sub(*LAST_IDLE_INSTANT);
    // *LAST_IDLE_INSTANT = now;
    // if dt > 72_000_000 * 2 {
    //     if *LAST_IDLE_COUNTER == *cx.resources.idle_counter {
    //         rprintln!(=>2, "IRQ lockup detected!");
    //         cx.resources.radio_irq.disable_interrupt(cx.resources.exti);
    //     }
    //     *LAST_IDLE_COUNTER = *cx.resources.idle_counter;
    // }
    //rprintln!(=>2, "IRQ: {}us", cycles2us!(cx, dt));
    // cx.resources.trace_pin.set_high().ok();
    cx.resources.radio.irq.clear_interrupt_pending_bit();

    for _ in 0..42 {
        radio::state_machine::advance(
            cx.resources.radio,
            &mut cx.resources.channels,
            buffer,
            &cx.spawn,
            &cx.schedule,
            cx.resources.clocks,
            cx.resources.scheduler,
            unsafe { &mut TRACER }
        );
        if cx.resources.radio.irq.is_low().unwrap() {
            break;
        }
    }
    if cx.resources.radio.irq.is_high().unwrap() {
        rprintln!(=>0, "radio_irq: still pending after many tries!");
    }
    // cx.resources.trace_pin.set_low().ok();
}

#[derive(Default)]
pub struct EventStateData {
    time_mark: Option<CycntInstant>,

    gts_processing_finished: bool,
    gts_duration: MicroSeconds,
    gts_started: Option<CycntInstant>,
    gts_elapsed: MicroSeconds,

    dyn_processing_finished: bool,
    dyn_duration: MicroSeconds,
    dyn_started: Option<CycntInstant>,
    dyn_elapsed: MicroSeconds,
}

impl EventStateData {
    pub fn clear_flags(&mut self) {
        self.gts_processing_finished = false;
        self.dyn_processing_finished = false;
    }
}

fn overlap_check(
    radio_commands: &mut crate::radio::types::CommandQueueP,
    done: bool,
    should_have_taken: MicroSeconds,
    actually_took: MicroSeconds,
    name: &'static str)
{
    if !done {
        // Message may still be sending, stop it not to overlap with anyone. This should not normally happen.
        // radio_commands.enqueue(Command::ForceReadyIfSending).ok();
        // rtic::pend(config::DW1000_IRQ_EXTI);
        //rprintln!(=>2, "{}{} slot overlap detected{}", color::YELLOW, name, color::DEFAULT);
    }
    let time_exceeded = actually_took > should_have_taken;
    if time_exceeded && done {
        // irq processing probably took too long, message could have overlapped with other slot.
        // rprintln!(=>2,
        //     "{}{} slot hard overlap detected by={} already_sent={}{}",
        //     color::RED,
        //     name,
        //     actually_took - should_have_taken,
        //     done,
        //     color::DEFAULT
        // );
    }
}

pub fn radio_event(mut cx: crate::radio_event::Context, e: Event) {
    // if let Event::GTSAboutToStart(_, _) = e {
    //     rprintln!(=>2, "\n\n\n---\n");
    // }
    // rprintln!(=>2, "e: {}\n", e);
    use Event::*;
    match e {
        #[cfg(feature = "master")]
        GTSAboutToStart(gt_phase_duration, radio_config) => {
            radio_command!(cx, Command::GTSStart(radio_config));
            let dt: MicroSeconds = config::GTS_PERIOD.into();
            cx.schedule.radio_event(
                cx.scheduled + us2cycles!(cx.resources.clocks, dt.0),
                GTSAboutToStart(Scheduler::gts_phase_duration(), radio_config)
            ).ok(); // TODO: count
            cx.resources.event_state_data.clear_flags();
            cx.resources.event_state_data.gts_duration = gt_phase_duration;
        },
        #[cfg(feature = "master")]
        TimeMark(instant) => {
            cx.resources.event_state_data.time_mark = Some(instant);
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSAboutToStart(gt_slot_duration, radio_config) => {
            radio_command!(cx, Command::SendGTSAnswer(gt_slot_duration, radio_config));
            cx.resources.event_state_data.clear_flags();
            cx.resources.event_state_data.gts_duration = gt_slot_duration;
            cx.resources.event_state_data.gts_started = Some(CycntInstant::now());
        },
        GTSProcessingFinished => {
            cx.resources.event_state_data.gts_processing_finished = true;
            let gts_started = cx.resources.event_state_data.gts_started;
            if gts_started.is_some() {
                let elapsed = CycntInstant::now().duration_since(gts_started.unwrap());
                cx.resources.event_state_data.gts_elapsed = us(cycles2us!(cx.resources.clocks, elapsed.as_cycles()) as u32);
            }
        },
        GTSShouldHaveEnded => {
            let elapsed = cx.resources.event_state_data.gts_elapsed;
            // rprintln!(=>2, "G_SX: {}\n", elapsed);
            overlap_check(
                &mut cx.resources.radio_commands,
                cx.resources.event_state_data.gts_processing_finished,
                cx.resources.event_state_data.gts_duration,
                elapsed,
                "GTS"
            );
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSStartReceived(time_mark) => {
            // Save timing marker
            cx.resources.event_state_data.time_mark = Some(time_mark);

            let half_guard: MicroSeconds = Scheduler::half_guard();
            let half_guard = us2cycles!(cx.resources.clocks, half_guard.0);
            let quarter_guard: MicroSeconds = Scheduler::quarter_guard();
            let quarter_guard = us2cycles!(cx.resources.clocks, quarter_guard.0);
            // Schedule the start of GTS (if any) or send answer right away if shift is 0.
            // Remove the time passed since actual message reception.
            // processing_delay is time passed since dw1000 timestamp and message actually pulled from it,
            // in cpu clock cycles.
            let gt_slot = cx.resources.radio.lock(|radio| radio.master.find_slot(SlotType::GtsUplink));
            match gt_slot {
                Some(s) => {
                    let duration: MicroSeconds = s.duration;
                    let duration = us2cycles!(cx.resources.clocks, duration.0);
                    if s.shift == MicroSeconds(0) {
                        cx.resources.event_state_data.gts_started = Some(CycntInstant::now());
                        radio_command!(cx, Command::SendGTSAnswer(s.duration, s.radio_config));

                        cx.schedule.radio_event(
                            time_mark + duration + half_guard,
                            Event::GTSShouldHaveEnded
                        ).ok(); // TODO: count
                    } else {
                        let shift: MicroSeconds = s.shift;
                        let shift = us2cycles!(cx.resources.clocks, shift.0);
                        cx.schedule.radio_event(
                            time_mark + shift,
                            Event::GTSAboutToStart(s.duration, s.radio_config)
                        ).ok(); // TODO: count

                        cx.schedule.radio_event(
                            time_mark + shift + duration + half_guard,
                            Event::GTSShouldHaveEnded
                        ).ok(); // TODO: count
                    }
                },
                None => {}
            }

            // Schedule the start of Aloha slot, if any.
            let aloha_slot = cx.resources.radio.lock(|radio| radio.master.find_slot(SlotType::Aloha) );
            match aloha_slot {
                Some(s) => {
                    let shift: MicroSeconds = s.shift;
                    let shift = us2cycles!(cx.resources.clocks, shift.0);
                    let duration: MicroSeconds = s.duration;
                    let duration = us2cycles!(cx.resources.clocks, duration.0);
                    cx.schedule.radio_event(
                        time_mark + shift,
                        Event::AlohaSlotAboutToStart(s.duration, s.radio_config)
                    ).ok(); // TODO: count
                    cx.schedule.radio_event(
                        time_mark + shift + duration + quarter_guard,
                        Event::AlohaSlotEnded
                    ).ok(); // TODO: count
                },
                None => {}
            }

            // Schedule the start of dyn slot, if any.
            let dyn_slot = cx.resources.radio.lock(|radio| radio.master.find_slot(SlotType::DynUplink) );
            match dyn_slot {
                Some(s) => {
                    let shift: MicroSeconds = s.shift;
                    let shift = us2cycles!(cx.resources.clocks, s.shift.0);
                    let duration: MicroSeconds = s.duration;
                    let duration = us2cycles!(cx.resources.clocks, duration.0);
                    //rprintln!(=>13, "till: {}", till_dyn_start);
                    cx.schedule.radio_event(
                        time_mark + shift,
                        Event::DynUplinkAboutToStart(s.duration, s.radio_config)
                    ).ok(); // TODO: count
                    cx.schedule.radio_event(
                        time_mark + shift + duration + half_guard,
                        Event::DynShouldHaveEnded
                    ).ok(); // TODO: count
                },
                None => {}
            }

            // Schedule the start of Ranging slot, if any.
            // let ranging_slot = cx.resources.radio.lock(|radio| radio.master.find_slot(SlotType::Ranging) );
            // match ranging_slot {
            //     Some(s) => {
            //         let shift: MicroSeconds = s.shift;
            //         let shift = us2cycles!(cx.resources.clocks, s.shift.0);
            //         let duration: MicroSeconds = s.duration;
            //         let duration = us2cycles!(cx.resources.clocks, duration.0);
            //         cx.schedule.radio_event(
            //             time_mark + shift,
            //             Event::RangingSlotAboutToStart(s.duration, s.radio_config)
            //         ).ok(); // TODO: count
            //         cx.schedule.radio_event(
            //             time_mark + shift + duration + quarter_guard,
            //             Event::RangingSlotEnded
            //         ).ok(); // TODO: count
            //     },
            //     None => {}
            // }

            // Schedule the listen command just before the next GTSStart is about to be sent from master
            let dt: MilliSeconds = config::GTS_PERIOD - MilliSeconds(2); // TODO: improve
            cx.schedule.radio_event(
                cx.scheduled + ms2cycles!(cx.resources.clocks, dt.0),
                Event::GTSStartAboutToBeBroadcasted
            ).ok(); // TODO: count
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        GTSStartAboutToBeBroadcasted => {
            radio_command!(cx, Command::ListenForGTSStart(RadioConfig::fast()));
            // cx.schedule.radio_event(
            //     cx.scheduled + ms2cycles!(cx.resources.clocks, config::GTS_PERIOD.0),
            //     GTSStartAboutToBeBroadcasted
            // ).ok(); // TODO: count
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        ReceiveCheck => {
            rtic::pend(config::DW1000_IRQ_EXTI);
            let dt: MilliSeconds = config::DW1000_CHECK_PERIOD;
            cx.schedule.radio_event(
                cx.scheduled + ms2cycles!(cx.resources.clocks, dt.0),
                ReceiveCheck
            ).ok(); // TODO: count
        },
        AlohaSlotAboutToStart(aloha_slot_duration, radio_config) => {

        },
        AlohaSlotEnded => {
            radio_command!(cx, Command::ForceReady);
        },
        #[cfg(feature = "master")]
        DynUplinkAboutToStart(dyn_phase_duration, radio_config) => {
            cx.resources.event_state_data.dyn_duration = dyn_phase_duration;
            cx.resources.event_state_data.dyn_started = Some(CycntInstant::now());
            radio_command!(cx, Command::DynListen(dyn_phase_duration, radio_config));
        },
        #[cfg(any(feature = "slave", feature = "anchor"))]
        DynUplinkAboutToStart(dyn_slot_duration, radio_config) => {
            cx.resources.event_state_data.dyn_duration = dyn_slot_duration;
            cx.resources.event_state_data.dyn_started = Some(CycntInstant::now());
            radio_command!(cx, Command::DynSend(dyn_slot_duration, radio_config));
        },
        DynProcessingFinished => {
            cx.resources.event_state_data.dyn_processing_finished = true;
            let dyn_started = cx.resources.event_state_data.dyn_started;
            if dyn_started.is_some() {
                let elapsed = CycntInstant::now().duration_since(dyn_started.unwrap());
                cx.resources.event_state_data.dyn_elapsed = us(cycles2us!(cx.resources.clocks, elapsed.as_cycles()) as u32);
            }
        },
        DynShouldHaveEnded => {
            let elapsed = cx.resources.event_state_data.dyn_elapsed;
            // rprintln!(=>2, "D_SX: {}\n", elapsed);
            overlap_check(
                &mut cx.resources.radio_commands,
                cx.resources.event_state_data.dyn_processing_finished,
                cx.resources.event_state_data.dyn_duration,
                elapsed,
                "Dyn"
            );
        },
        RangingSlotAboutToStart(ranging_slot_duration, radio_config) => {
            radio_command!(cx, Command::RangingStart(ranging_slot_duration, radio_config));
        },
        RangingSlotEnded => {
            radio_command!(cx, Command::ForceReady);
        }
    }
}

use crate::config;
use crate::radio;
use crate::motion;
use crate::board::hal;
use crate::radio::types::{Command, Event, RadioConfig};
use rtic::Mutex;
use rtt_target::{ rprint, rprintln };
use rtic::cyccnt::U32Ext;
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
        let gpioa = device.GPIOA;
        let count = e as u8;
        for _ in 0..count {
            gpioa.bsrr.write(|w| w.bs1().set_bit());
            cortex_m::asm::nop();
            gpioa.bsrr.write(|w| w.br1().set_bit());
        }

        let dt_gts = now - self.prev_gts;
        let dt_prev = now - self.prev;
        if e == TraceEvent::GTSStart {
            rprintln!(=> 13, "\n\n\n---\n");
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

pub struct NoOpTracer {}

impl Tracer for NoOpTracer {
    fn event(&mut self, _e: TraceEvent) { }
}

static mut TRACER: RttTracer = RttTracer { prev: 0, prev_gts: 0, sysclk: 72_000_000 };

pub fn radio_irq(cx: crate::radio_irq::Context, buffer: &mut[u8], ) {
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

    cx.resources.radio.irq.clear_interrupt_pending_bit();
    for _ in 0..42 {
        radio::state_machine::advance(
            cx.resources.radio,
            cx.resources.channels,
            buffer,
            &cx.spawn,
            &cx.schedule,
            cx.resources.clocks,
            unsafe { &mut TRACER }
        );
        if cx.resources.radio.irq.is_low().unwrap() {
            break;
        }
    }
    if cx.resources.radio.irq.is_high().unwrap() {
        rprintln!(=>2, "radio_irq: still pending after many tries!");
    }
}

#[derive(Default)]
pub struct EventStateData {
    time_marker: Option<CycntInstant>,
    gts_processing_finished: bool,
    gts_duration: MicroSeconds,
    dyn_processing_finished: bool,
    dyn_duration: MicroSeconds,
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
        radio_commands.enqueue(Command::ForceReadyIfSending).ok();
        rtic::pend(config::DW1000_IRQ_EXTI);
        rprintln!(=>2, "{}slot overlap detected, {}", color::YELLOW, color::DEFAULT);
    }
    let time_exceeded = actually_took > should_have_taken;
    if time_exceeded {
        // irq processing probably took too long, message could have overlapped with other slot.
        rprintln!(=>2,
            "{}slot hard overlap detected by={} already_sent={}{}",
            color::RED,
            actually_took - should_have_taken,
            done,
            color::DEFAULT
        );
    }
}

pub fn radio_event(mut cx: crate::radio_event::Context, e: Event) {
    if let Event::GTSAboutToStart(_) = e {
        rprintln!(=>2, "\n\n\n---\n");
    }
    rprintln!(=>2, "e: {}\n", e);
    use Event::*;
    match e {
        #[cfg(feature = "master")]
        GTSAboutToStart(gt_phase_duration) => {
            radio_command!(cx, Command::GTSStart);
            let dt: MicroSeconds = config::GTS_PERIOD.into();
            cx.schedule.radio_event(
                cx.scheduled + us2cycles!(cx.resources.clocks, dt.0),
                GTSAboutToStart(Scheduler::gts_phase_duration())
            ).ok(); // TODO: count
            cx.resources.event_state_data.clear_flags();
            cx.resources.event_state_data.gts_duration = gt_phase_duration;
        },
        #[cfg(feature = "master")]
        TimeMarker(instant) => {
            cx.resources.event_state_data.time_marker = Some(instant);
        },
        #[cfg(feature = "slave")]
        GTSAboutToStart(gt_slot_duration) => {
            radio_command!(cx, Command::SendGTSAnswer(gt_slot_duration));
            cx.resources.event_state_data.clear_flags();
            cx.resources.event_state_data.gts_duration = gt_slot_duration;
        },
        GTSProcessingFinished => {
            cx.resources.event_state_data.gts_processing_finished = true;
        },
        GTSShouldHaveEnded => {
            let time_marker = cx.resources.event_state_data.time_marker;
            if time_marker.is_some() {
                let elapsed = cx.resources.event_state_data.time_marker.unwrap().elapsed();
                let elapsed: MicroSeconds = us(cycles2us!(cx.resources.clocks, elapsed.as_cycles()) as u32);
                overlap_check(
                    &mut cx.resources.radio_commands,
                    cx.resources.event_state_data.gts_processing_finished,
                    cx.resources.event_state_data.gts_duration,
                    elapsed,
                    "GTS"
                );
            }
        },
        #[cfg(feature = "slave")]
        GTSStartReceived(processing_delay) => {
            // Save timing marker
            cx.resources.event_state_data.time_marker = Some(cx.scheduled - processing_delay);

            // Schedule the start of GTS (if any) or send answer right away if shift is 0.
            // Remove the time passed since actual message reception.
            // processing_delay is time passed since dw1000 timestamp and message actually pulled from it,
            // in cpu clock cycles.
            let gt_slot = cx.resources.radio.lock(|radio| radio.master.gt_slot());
            match gt_slot {
                Some(s) => {
                    let duration: MicroSeconds = s.duration;
                    let duration = us2cycles_raw!(cx.resources.clocks, duration.0);
                    if s.shift == MicroSeconds(0) {
                        radio_command!(cx, Command::SendGTSAnswer(s.duration));

                        let instant = cx.scheduled + duration.cycles();
                        cx.schedule.radio_event(instant, Event::GTSShouldHaveEnded).ok(); // TODO: count
                    } else {
                        let shift: MicroSeconds = s.shift;
                        let till_gts = us2cycles_raw!(cx.resources.clocks, shift.0) - processing_delay.as_cycles();
                        cx.schedule.radio_event(cx.scheduled + till_gts.cycles(), Event::GTSAboutToStart(s.duration)).ok(); // TODO: count

                        let instant = cx.scheduled + till_gts.cycles() + duration.cycles();
                        cx.schedule.radio_event(instant, Event::GTSShouldHaveEnded).ok(); // TODO: count
                    }
                },
                None => {}
            }

            // Schedule the start of Aloha slot, if any.
            let aloha_slot = cx.resources.radio.lock(|radio| radio.master.aloha_slot() );
            match aloha_slot {
                Some(s) => {
                    let till_aloha = us2cycles_raw!(cx.resources.clocks, s.shift.0) - processing_delay.as_cycles();
                    cx.schedule.radio_event(cx.scheduled + till_aloha.cycles(), Event::AlohaSlotAboutToStart(s.duration)).ok(); // TODO: count
                    cx.schedule.radio_event(cx.scheduled + till_aloha.cycles(), Event::AlohaSlotEnded).ok(); // TODO: count
                },
                None => {}
            }

            // Schedule the start of dyn slot, if any.
            let dyn_slot = cx.resources.radio.lock(|radio| radio.master.dyn_slot() );
            match dyn_slot {
                Some(s) => {
                    let till_dyn_start = us2cycles_raw!(cx.resources.clocks, s.shift.0) - processing_delay.as_cycles();
                    let dyn_duration = us2cycles_raw!(cx.resources.clocks, s.duration.0);
                    //rprintln!(=>13, "till: {}", till_dyn_start);
                    cx.schedule.radio_event(
                        cx.scheduled + till_dyn_start.cycles(),
                        Event::DynAboutToStart(s.duration)
                    ).ok(); // TODO: count
                    cx.schedule.radio_event(
                        cx.scheduled + till_dyn_start.cycles() + dyn_duration.cycles(),
                        Event::DynShouldHaveEnded
                    ).ok(); // TODO: count
                },
                None => {}
            }

            // Schedule the listen command just before the next GTSStart is about to be sent from master
            let dt: MilliSeconds = config::GTS_PERIOD - MilliSeconds(2); // TODO: improve
            cx.schedule.radio_event(
                cx.scheduled + ms2cycles!(cx.resources.clocks, dt.0),
                Event::GTSStartAboutToBeBroadcasted
            ).ok(); // TODO: count
        },
        #[cfg(feature = "slave")]
        GTSStartAboutToBeBroadcasted => {
            radio_command!(cx, Command::ListenForGTSStart(RadioConfig::default()));
        },
        #[cfg(feature = "slave")]
        ReceiveCheck => {
            rtic::pend(config::DW1000_IRQ_EXTI);
            let dt: MilliSeconds = config::DW1000_CHECK_PERIOD;
            cx.schedule.radio_event(
                cx.scheduled + ms2cycles!(cx.resources.clocks, dt.0),
                ReceiveCheck
            ).ok(); // TODO: count
        },
        AlohaSlotAboutToStart(aloha_slot_duration) => {

        },
        AlohaSlotEnded => {
            radio_command!(cx, Command::ForceReady);
        },
        #[cfg(feature = "master")]
        DynAboutToStart(dyn_phase_duration) => {
            cx.resources.event_state_data.dyn_duration = dyn_phase_duration;
        },
        #[cfg(feature = "slave")]
        DynAboutToStart(dyn_slot_duration) => {
            cx.resources.event_state_data.dyn_duration = dyn_slot_duration;
        },
        DynProcessingFinished => {
            cx.resources.event_state_data.dyn_processing_finished = true;
        },
        DynShouldHaveEnded => {
            let time_marker = cx.resources.event_state_data.time_marker;
            if time_marker.is_some() {
                let elapsed = time_marker.unwrap().elapsed();
                let elapsed: MicroSeconds = us(cycles2us!(cx.resources.clocks, elapsed.as_cycles()) as u32);
                overlap_check(
                    &mut cx.resources.radio_commands,
                    cx.resources.event_state_data.dyn_processing_finished,
                    cx.resources.event_state_data.dyn_duration,
                    elapsed,
                    "Dyn"
                );
            }
        }
    }
}

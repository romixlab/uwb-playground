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

pub enum RadioChronoState {
    Uninit,
    Idle,
    #[cfg(feature = "master")]
    GTSInProgress,
}

#[allow(unused_variables)]
pub fn radio_chrono(mut cx: crate::radio_chrono::Context, state: &mut RadioChronoState) {
    #[cfg(feature = "master")]
    const GTS_END_MS: u32 = 30;
    use RadioChronoState::*;
    *state = match state {
        Uninit => {
            radio_command!(cx, Command::Listen(RadioConfig::default()));
            #[cfg(feature = "slave")]
            cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok();
            #[cfg(feature = "master")]
            cx.spawn.radio_chrono().ok();
            Idle
        },
        Idle => {
            cfg_if! {
                if #[cfg(feature = "master")] {
                    unsafe { TRACER.event(TraceEvent::GTSChronoStart); }
                    radio_command!(cx, Command::GTSStart);

                    cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, GTS_END_MS)).ok(); // TODO: Count errors
                    GTSInProgress
                } else if #[cfg(feature = "slave")] {
                    rtic::pend(config::DW1000_IRQ_EXTI);
                    cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok();
                    Idle
                }
            }
        },
        #[cfg(feature = "master")]
        GTSInProgress => {
            unsafe { TRACER.event(TraceEvent::GTSChronoEnd); }
            radio_command!(cx, Command::GTSEnd);

            cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS - GTS_END_MS)).ok(); // TODO: Count errors
            Idle
        }
    };
}

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
        if e == TraceEvent::GTSChronoStart || e == TraceEvent::GTSStartReceived {
            self.prev_gts = now;
            rprintln!(=> 13, "\n\n\n---\n");
        }
        self.prev = now;

        let dt_gts = cycles2us_raw!(self.sysclk, dt_gts);
        let dt_prev = cycles2us_raw!(self.sysclk, dt_prev);
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

pub fn radio_event(mut cx: crate::radio_event::Context, e: Event) {
    rprintln!(=>2, "e: {:?}\n", e);
    use Event::*;
    match e {
        // #[cfg(feature = "master")]
        // GTSAnswerReceived(from, answer) => {
        //     use crate::motion::Tachometer;
        //
        //     //let stat = cx.resources.stat;
        //     cx.resources.mecanum_wheels.lock(|wheels| {
        //         if from == config::TR_UWB_ADDR {
        //             wheels.top_right.tacho = Tachometer(answer.data.tacho);
        //             wheels.top_right.power_in = answer.data.power_in;
        //             //stat.tr_gts_answers += 1;
        //         } else if from == config::BL_UWB_ADDR {
        //             wheels.bottom_left.tacho = Tachometer(answer.data.tacho);
        //             wheels.bottom_left.power_in = answer.data.power_in;
        //             //stat.bl_gts_answers += 1;
        //         } else if from == config::BR_UWB_ADDR {
        //             wheels.bottom_right.tacho = Tachometer(answer.data.tacho);
        //             wheels.bottom_right.power_in = answer.data.power_in;
        //             //stat.br_gts_answers += 1;
        //         }
        //     });
        // },
        // #[cfg(feature = "devnode")]
        // GTSAnswerReceived(from, answer) => {
        //     rprintln!(=>2, "GTS answer rx from: {:?}, tacho: {}, pwr: {}", from, answer.data.tacho, answer.data.power_in);
        // },
        #[cfg(feature = "slave")]
        GTSStartReceived => {
            // Prepare telemetry for sending in a given slot
            // let tacho = cx.resources.wheel.tacho;
            // let power_in = cx.resources.wheel.power_in;
            // let uplink_data = radio::message::GTSUplinkData { tacho: tacho.0, power_in };
            // let gts_answer = radio::message::GTSAnswer { data: uplink_data };
            // // cx.resources.radio_commands_p.lock(|commands|
            // //     commands.enqueue(radio::Command::GTSSendAnswer(tx_time, gts_answer))).ok(); // TODO: Count errors
            // // Schedule an rpm sending after all GTS, so that each MC receive the command at the same time
            // let rpm = motion::Rpm(gts_entry.sync_no_ack_data.rpm);
            // cx.resources.wheel.rpm = rpm;
            //cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 as u32 / 1000), radio::types::Event::GTSEnded).ok(); // TODO: count errors;
            radio_command_l!(cx, Command::SendGTSAnswer);
        },
        #[cfg(feature = "slave")]
        GTSAnswerSent => {
            radio_command_l!(cx, Command::Listen(RadioConfig::default()));

            let dyn_window = cx.resources.radio.lock(|radio| {
                match radio.master {
                    NodeState::Disconnected => { None },
                    NodeState::Active(node) => {
                        let when_gts_started = node.last_seen.unwrap().0;
                        let from_gts_start = {
                            if node.slots[2].is_some() {
                                node.slots[2].unwrap().shift
                            } else if node.slots[1].is_some() {
                                node.slots[1].unwrap().shift
                            } else {
                                return None;
                            }
                        };
                        Some((when_gts_started, from_gts_start))
                    }
                }
            });

            match dyn_window {
                Some((when_gts_started, from_gts_start)) => {
                    let since_gts_start = (cx.scheduled - when_gts_started).as_cycles();
                    let till_dyn_start = us2cycles_raw!(cx, from_gts_start.0) - since_gts_start;
                    cx.schedule.radio_event(cx.scheduled + till_dyn_start.cycles(), Event::DynWindowStarted);
                },
                None => {}
            }
        },
        DynWindowStarted => {
            radio_command_l!(cx, Command::DynWindowStart);
        },
        GTSEnded => {
            cfg_if::cfg_if! {
                    if #[cfg(feature = "master")] {
                        let rpm = cx.resources.mecanum_wheels.lock(|wheels| wheels.top_left.rpm);
                        cx.spawn.ctrl_link_control().ok(); // TODO: count errors;
                    } else if  #[cfg(feature = "slave")] {
                        let rpm = cx.resources.wheel.rpm;
                        cx.spawn.motor_control(motion::MotorControlEvent::SetRpm(rpm)).ok(); // TODO: count errors
                    }
                }
            //cx.spawn.motor_control(MotorControlEvent::SetRpm(rpm)).ok(); // TODO: count errors
            cx.spawn.motor_control(motion::MotorControlEvent::RequestTelemetry).ok(); // TODO: count errors
        }
    }
}

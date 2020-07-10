use crate::config;
use crate::radio;
use crate::motion;
use crate::board::hal;

use rtic::Mutex;
use rtt_target::{ rprint, rprintln };
use rtic::cyccnt::U32Ext;
use hal::gpio::ExtiPin;
use embedded_hal::digital::v2::{OutputPin, InputPin, ToggleableOutputPin};
use crate::radio::{Arbiter, LogicalDestination, ChannelId, Multiplex};
use crate::motion::MoveCommand;
use crate::util::{Tracer, TraceEvent};

pub enum RadioChronoState {
    Idle,
    #[cfg(feature = "master")]
    GTSInProgress,
}

/// **Reliability**
/// * `R` - (Reliable) ACKed, with retransmission (as in TCP).
/// * `N` - NACKed, with retransmission (as in CoAP).
/// * `I` - Ignore lost bytes/frames (as in UDP).
///
/// **Queueing**
/// * `0` - Unbuffered.
/// * `P` - Packetized queue with fixed or dynamic frames.
/// * `B` - Byte buffer or Stream.
///
/// **Ordering**
/// * `L` - LIFO (need newest data).
/// * `F` - FIFO (need sequential data).
///
/// **Timing**
/// * `A` - Asynchronous, sent some time in the future.
/// * `T` - Timed / synchronous, sent in a predetermined window.
///
/// **Overflow behaviour**
/// * `D` - Drop on overflow.
/// * `G` - Stop on overflow (gave up).
/// * `P` - Panic on overflow.
///
/// **Security**
/// * `E` - Encrypted.
/// * `H` - Hackable.
///
/// **Duplicates**
/// * `S` - Sequence numbered, repeated data dropped (copies may arrive as in UDP or redundant systems).
/// * `U` - Unique, channel does not create copies.
///
/// **Priority**
/// * `1` and up, default is 1. Equal priorities are round-robin scheduled. Lowest priority sync is higher than any other async.
///
/// **Channels**
/// * `@ID` - data is grabbed from specific queue and enqueued to the same id on the other node.
pub struct DataQueues {
    ///// I/0/_/T/G/H/_/10 @66
    //pub stop_command: Option<Stop>,
    //#[queue(I/0/L/T/D/H/S/9 @70)]
    pub move_command: Option<MoveCommand>,
    ///// I/0/L/T/D/H/S/9 @71
    //pub odometry: Option<OdometryData>
    ///// I/0/_/T/D/H/S/8 @72
    //pub heartbeat: Option<Heartbeat>

    //#[cfg(any(feature = "master", feature = "br"))]
    ///// I/P/L/A/D/H/U/10 @170
    //pub lidar_data: crate::rplidar::LidarQueue,

    ///// R/P/F/A/D/H/S/9 @171
    // pub reqrep

}

impl DataQueues {
    pub fn new() -> Self {
        DataQueues {
            move_command: None,
        }
    }
}

impl Arbiter for DataQueues {
    fn source_sync<M: Multiplex>(&mut self, mux: &mut M)
    {

    }

    fn source_async<M: Multiplex>(&mut self, mux: &mut M) {
        unimplemented!()
    }

    fn sink_async(&mut self, channel: ChannelId, chunk: &[u8]) {
        unimplemented!()
    }

    fn sink_sync(&mut self, channel: ChannelId, chunk: &[u8]) {
        rprint!(=>1, "arb: CH{}:[", channel);
        for b in chunk {
            rprint!(=>1, "{:02x} ", b);
        }
        rprintln!(=>1, "]\n");
    }
}

#[allow(unused_variables)]
pub fn radio_chrono(mut cx: crate::radio_chrono::Context, state: &mut RadioChronoState) {
    #[cfg(feature = "master")]
    const GTS_END_MS: u32 = 30;
    //use radio::Command;
    //use radio::message::*;
    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                //let now = DWT::get_cycle_count();

                use crate::radio::message::{GTSEntry, GTSStart, GTSDownlinkData};
                use crate::radio::{Command};
                use RadioChronoState::*;
                *state = match state {
                    Idle => {
                        unsafe { TRACER.event(TraceEvent::GTSChronoStart); }
                        cx.resources.radio_command.lock(|cmd| *cmd = Some(Command::GTSStart));
                        rtic::pend(config::DW1000_IRQ_EXTI);

                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, GTS_END_MS)).ok(); // TODO: Count errors
                        RadioChronoState::GTSInProgress
                    },
                    GTSInProgress => {
                        unsafe { TRACER.event(TraceEvent::GTSChronoEnd); }
                        cx.resources.radio_command.lock(|cmd| *cmd = Some(Command::GTSEnd));
                        rtic::pend(config::DW1000_IRQ_EXTI);

                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS - GTS_END_MS)).ok(); // TODO: Count errors
                        Idle
                    }
                }
            } else if #[cfg(all(feature = "slave", not(feature = "devnode")))] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok(); // TODO: count errors
            } else if #[cfg(feature = "devnode")] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok(); // TODO: count errors
            }
        }
    rtic::pend(config::DW1000_IRQ_EXTI);
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

        // let dt_gts = now - self.prev_gts;
        // let dt_prev = now - self.prev;
        // if e == TraceEvent::GTSChronoStart {
        //     self.prev_gts = now;
        //     rprintln!(=> 13, "\n\n\n---\n");
        // }
        // self.prev = now;
        //
        // let dt_gts = cycles2us_raw!(self.sysclk, dt_gts);
        // let dt_prev = cycles2us_raw!(self.sysclk, dt_prev);
        // rprintln!(=> 13,
        //     "{}({}) +{}.{:03} d{}.{:03}\n",
        //     e,
        //     count,
        //     dt_gts / 1000, dt_gts % 1000,
        //     dt_prev / 1000, dt_prev % 1000
        // );
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
    //         rprintln!("IRQ lockup detected!");
    //         cx.resources.radio_irq.disable_interrupt(cx.resources.exti);
    //     }
    //     *LAST_IDLE_COUNTER = *cx.resources.idle_counter;
    // }
    //rprintln!("IRQ: {}us", cycles2us!(cx, dt));

    cx.resources.radio_irq.clear_interrupt_pending_bit();
    for _ in 0..42 {
        radio::state_machine::advance(
            cx.resources.radio_state,
            cx.resources.radio_queues,
            cx.resources.radio_command,
            buffer,
            &cx.spawn,
            unsafe { &mut TRACER }
        );
        if cx.resources.radio_irq.is_low().unwrap() {
            break;
        }
    }
    if cx.resources.radio_irq.is_high().unwrap() {
        rprintln!("radio_irq: still pending after many tries!");
    }
}

pub fn radio_event(mut cx: crate::radio_event::Context, e: radio::Event) {
    rprintln!("radio_event: {:?}\n", e);
    use radio::Event::*;
    match e {
        #[cfg(feature = "master")]
        GTSAnswerReceived(from, answer) => {
            use crate::motion::Tachometer;

            let stat = cx.resources.stat;
            cx.resources.mecanum_wheels.lock(|wheels| {
                if from == config::TR_UWB_ADDR {
                    wheels.top_right.tacho = Tachometer(answer.data.tacho);
                    wheels.top_right.power_in = answer.data.power_in;
                    stat.tr_gts_answers += 1;
                } else if from == config::BL_UWB_ADDR {
                    wheels.bottom_left.tacho = Tachometer(answer.data.tacho);
                    wheels.bottom_left.power_in = answer.data.power_in;
                    stat.bl_gts_answers += 1;
                } else if from == config::BR_UWB_ADDR {
                    wheels.bottom_right.tacho = Tachometer(answer.data.tacho);
                    wheels.bottom_right.power_in = answer.data.power_in;
                    stat.br_gts_answers += 1;
                }
            });
        },
        #[cfg(feature = "devnode")]
        GTSAnswerReceived(from, answer) => {
            rprintln!("GTS answer rx from: {:?}, tacho: {}, pwr: {}", from, answer.data.tacho, answer.data.power_in);
        },
        #[cfg(all(feature = "slave", not(feature = "devnode")))]
        GTSStartReceived(tx_time, gts_end_dt, gts_entry) => {
            // Prepare telemetry for sending in a given slot
            let tacho = cx.resources.wheel.tacho;
            let power_in = cx.resources.wheel.power_in;
            let uplink_data = radio::message::GTSUplinkData { tacho: tacho.0, power_in };
            let gts_answer = radio::message::GTSAnswer { data: uplink_data };
            // cx.resources.radio_commands_p.lock(|commands|
            //     commands.enqueue(radio::Command::GTSSendAnswer(tx_time, gts_answer))).ok(); // TODO: Count errors
            // Schedule an rpm sending after all GTS, so that each MC receive the command at the same time
            let rpm = motion::Rpm(gts_entry.sync_no_ack_data.rpm);
            cx.resources.wheel.rpm = rpm;
            cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 / 1000), radio::Event::GTSEnded).ok(); // TODO: count errors;
        },
        #[cfg(feature = "devnode")]
        GTSStartReceived(tx_time, gts_end_dt, gts_entry) => {
            rprintln!("GTS start received");
            cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 / 1000), radio::Event::GTSEnded);
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

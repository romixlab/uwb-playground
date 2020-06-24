#![no_main]
#![no_std]

mod board;
mod panic_handler;
mod radio;
#[macro_use]
mod util;
mod config;
mod crc_framer;

use dw1000::{
    DW1000,
    mac
};
use radio::{
    RadioState,
};

// use core::sync::atomic::{self, Ordering};
use rtic::{app};
use rtic::cyccnt::{U32Ext};
use crate::board::hal;
use hal::prelude::*;
use hal::gpio::{PushPull, Output, Input, PullDown};
#[cfg(feature = "pozyx-board")]
use hal::gpio::{Edge, ExtiPin};
#[cfg(feature = "pozyx-board")]
use hal::gpio::{gpioa::{PA0, PA1}, gpiob::{PB5,}};
#[cfg(feature = "dragonfly-board")]
use hal::gpio::{gpiob::{PB12, PB13, PB14, PB15}, gpioc::{PC8, PC9, PC10, }, };
use rtt_target::{rtt_init_print, rprintln, rprint};
use cortex_m::peripheral::DWT;
use hal::{
    spi::Spi,
};
use embedded_hal::spi::MODE_0;
use embedded_hal::digital::v2::OutputPin;
use hal::rcc::Clocks;
use core::num::Wrapping;
use bbqueue::{BBBuffer, ConstBBBuffer};

// enum TracePin<O> {
//     Dummy,
//     Output(O)
// }
//
// impl<O> TracePin<O>
//     where O: OutputPin
// {
//
// }

#[cfg(feature = "pozyx-board")]
type LedBlinkyPin = PB5<Output<PushPull>>;
#[cfg(feature = "dragonfly-board")]
type LedBlinkyPin = PC10<Output<PushPull>>;

#[cfg(feature = "pozyx-board")]
type RadioIrqPin = PA0<Input<PullDown>>;
//type RadioIrqPin = PC4<Input<PullDown>>;
#[cfg(feature = "dragonfly-board")]
type RadioIrqPin = PC9<Input<PullDown>>;

#[cfg(feature = "pozyx-board")]
type RadioTracePin = PA1<Output<PushPull>>;
#[cfg(feature = "dragonfly-board")]
type RadioTracePin = PC8<Output<PushPull>>;

use hal::gpio::gpiob::{PB6, PB7};
use hal::gpio::Alternate;
use hal::gpio::AF7;
use hal::stm32::Interrupt;

type VescTxPin = PB6<Alternate<AF7>>;
type VescRxPin = PB7<Alternate<AF7>>;
type VescSerial = hal::serial::Serial<hal::stm32::USART1, (VescTxPin, VescRxPin)>;
type VescBBBufferSize = generic_array::typenum::consts::U512;
type VescBBBufferP = bbqueue::Producer<'static, VescBBBufferSize>;
type VescBBBufferC = bbqueue::Consumer<'static, VescBBBufferSize>;
const VESC_IRQ_EXTI: Interrupt = Interrupt::USART1;
const VESC_RPM_ARRAY_FRAME_ID: u8 = 86;
const VESC_TACHO_ARRAY_FRAME_ID: u8 = 87;
const VESC_SETRPM_FRAME_ID: u8 = 8;
const VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID: u8 = 50;
const VESC_LIFT_FRAME_ID: u8 = 88;
const VESC_RESET_ALL: u8 = 89;
const VESC_REQUESTED_VALUES: u32 = (1 << 13) | (1 << 3) | (1 << 8); // tacho + i_in + v_in

use hal::gpio::gpioa::{PA2, PA3};
type CtrlTxPin = PA2<Alternate<AF7>>;
type CtrlRxPin = PA3<Alternate<AF7>>;
type CtrlSerial = hal::serial::Serial<hal::stm32::USART2, (CtrlTxPin, CtrlRxPin)>;
type CtrlBBBufferSize = generic_array::typenum::consts::U512;
type CtrlBBBufferP = bbqueue::Producer<'static, CtrlBBBufferSize>;
type CtrlBBBufferC = bbqueue::Consumer<'static, CtrlBBBufferSize>;
const CTRL_IRQ_EXTI: Interrupt = Interrupt::USART2;

use hal::gpio::gpioc::{PC6};
use hal::gpio::AF8;
use stm32f4xx_hal::serial::NoRx;
type LiftTxPin = PC6<Alternate<AF8>>;
type LiftSerial = hal::serial::Serial<hal::stm32::USART6, (LiftTxPin, NoRx)>;

/// Lost -> GTSAnswer received -> Verifying(+1) -> v:threshold reached -> Active(a:threshold) ->
/// -> GTSAnswer missing -> Active(-1) -> 0 reached -> Lost
pub enum SlaveState {
    Lost,
    Active(u8),
    Verifying(u8)
}

impl Default for SlaveState {
    fn default() -> Self {
        SlaveState::Lost
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct Rpm(pub i32);

#[derive(Default, Debug, Copy, Clone)]
pub struct Tachometer(pub i32);

#[derive(Default)]
pub struct MCData {
    /// 0 - no answer, +1 on answer from MC up to threshold, -1 on nack
    tokens: u8,
    rpm: Rpm,
    tacho: Tachometer,
    tacho_shift: i32,
    power_in: f32,
}

#[derive(Default)]
pub struct MecanumWheels {
    top_left: MCData,
    top_right: MCData,
    bottom_left: MCData,
    bottom_right: MCData
}

#[derive(Default, Debug)]
pub struct RpmArray {
    top_left: Rpm,
    top_right: Rpm,
    bottom_left: Rpm,
    bottom_right: Rpm
}

#[derive(Debug)]
pub struct MilliMeters(pub i32);
#[derive(Debug)]
pub struct Degrees(pub i32);

#[derive(Debug)]
pub struct XYThetaMove {
    dx: MilliMeters,
    dy: MilliMeters,
    dtheta: Degrees
}

#[derive(Debug)]
pub enum MotorControlEvent {
    #[cfg(feature = "master")]
    /// Received from Ctrl input, no override.
    /// Save values into resources.mecanum_wheels, they will be sent to slaves by radio_chrono task.
    SetRpmArray(RpmArray),
    #[cfg(feature = "master")]
    /// Received from dev uwb node, override ctrl input
    /// Displace and keep heading while rotating (normal car).
    /// Calculate rpms fromm displacement and do the same as SetRpmArray.
    MovePreserveHeading(XYThetaMove),
    #[cfg(feature = "master")]
    /// Received from dev uwb node, override ctrl input.
    /// Displace and ingore heading change caused by rotation (drifting car).
    /// Calculate rpms fromm displacement and do the same as SetRpmArray.
    MoveIgnoreHeading(XYThetaMove),
    /// Commit SET_RPM frame to vesc send bip buffer. Pend vesc_serial task to
    /// actually start sending it.
    SetRpm(Rpm),
    /// Commit GET_VALUES_SELECTIVE to vesc send bip buffer. Pend vesc_serial task to
    /// actually start sending it.
    RequestTelemetry,
    /// Quick hack
    ResetTacho // TODO: forward messages untouched and implement real reset
}

#[derive(Default)]
pub struct Slaves {
    top_right: SlaveState,
    bottom_left: SlaveState,
    bottom_right: SlaveState,
    dev: SlaveState
}

impl Slaves {
    pub fn new() -> Self {
        Slaves::default()
    }
}

enum RadioChronoState {
    Idle,
    GTSInProgress,
}

#[derive(Debug)]
enum LiftControlCommand {
    Continue,
    LeftUp,
    LeftDown,
    RightUp,
    RightDown,
    AllUp,
    AllDown,
    AllUpLong,
    AllDownLong,
    Stop
}

#[derive(Default)]
pub struct Stat {
    pub tr_gts_answers: u32,
    pub bl_gts_answers: u32,
    pub br_gts_answers: u32,
}

#[app(device = crate::board::hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,

        radio_state: RadioState,
        radio_irq: RadioIrqPin,
        radio_trace: RadioTracePin,
        radio_commands_p: radio::CommandQueueP,
        radio_commands_c: radio::CommandQueueC,
        #[cfg(feature = "master")]
        slaves: Slaves,

        vesc_serial: VescSerial,
        vesc_framer: crc_framer::CrcFramerDe<generic_array::typenum::consts::U512>,
        vesc_bbbuffer_p: VescBBBufferP,
        vesc_bbbuffer_c: VescBBBufferC,

        lift_serial: LiftSerial,

        ctrl_serial: CtrlSerial,
        ctrl_framer: crc_framer::CrcFramerDe<generic_array::typenum::consts::U512>,
        ctrl_bbbuffer_p: CtrlBBBufferP,
        ctrl_bbbuffer_c: CtrlBBBufferC,

        #[cfg(feature = "master")]
        mecanum_wheels: MecanumWheels,
        #[cfg(feature = "slave")]
        wheel: MCData,

        led_blinky: LedBlinkyPin,

        idle_counter: Wrapping<u32>,
        exti: hal::stm32::EXTI,

        #[cfg(feature = "master")]
        stat: Stat,
    }

    #[init(schedule = [], spawn = [radio_chrono, blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RADIO_COMMANDS_QUEUE: radio::CommandQueue = heapless::spsc::Queue(heapless::i::Queue::new());
        static mut VESC_BBBUFFER: BBBuffer<VescBBBufferSize> = BBBuffer(ConstBBBuffer::new());
        static mut CTRL_BBBUFFER: BBBuffer<CtrlBBBufferSize> = BBBuffer(ConstBBBuffer::new());

        rtt_init_print!(NoBlockSkip);
        rprintln!("\x1b[2J\x1b[0m");
        rprintln!("init()");

        let mut core/*: cortex_m::Peripherals */= cx.core;
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let device: hal::stm32::Peripherals = cx.device;
        //let _flash = device.FLASH;
        let rcc = device.RCC.constrain();
        cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let clocks = rcc.cfgr.sysclk(72.mhz()).freeze();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut flash = device.FLASH.constrain();
                let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);
                let clocks = rcc.cfgr.sysclk(72.mhz()).freeze(&mut flash.acr, &mut pwr);
            }
        }
        let mut syscfg = device.SYSCFG;
        let mut exti = device.EXTI;

        cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let gpioa = device.GPIOA.split();
                let gpiob = device.GPIOB.split();
                let gpioc = device.GPIOC.split();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);
                let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
                let mut gpioc = device.GPIOC.split(&mut rcc.ahb2);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let mut led1_red = gpiob.pb4.into_push_pull_output();
                let mut led_blinky = gpiob.pb5.into_push_pull_output();
                let mut led2_red = gpiob.pb8.into_push_pull_output();
                let mut led2_green = gpiob.pb9.into_push_pull_output();
                led1_red.set_low().ok();
                led2_green.set_high().ok();
                led2_red.set_low().ok();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut led_blinky = gpioc.pc10.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
            }
        }
        led_blinky.set_low().ok();

        // DW1000
        let dw1000_spi_freq = 1.mhz();

        cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let mut dw1000_reset  = gpiob.pb0.into_open_drain_output(); // open drain, do not pull high
                let mut dw1000_cs = gpioa.pa4.into_push_pull_output();
                let dw1000_clk    = gpioa.pa5.into_alternate_af5();
                let dw1000_mosi   = gpioa.pa7.into_alternate_af5();
                let dw1000_miso   = gpioa.pa6.into_alternate_af5();
                let _dw1000_wakeup = gpioc.pc5;
                let mut dw1000_irq = gpioa.pa0.into_pull_down_input(); // Header pin 2 jump wired to IRQ pin
                //let mut dw1000_irq    = gpioc.pc4.into_pull_down_input(); // IRQ never ends with this
                let trace_pin = gpioa.pa1.into_push_pull_output(); // Header pin 1
                dw1000_irq.make_interrupt_source(&mut syscfg);
                dw1000_irq.trigger_on_edge(&mut exti, Edge::RISING);
                dw1000_irq.enable_interrupt(&mut exti);

                let dw1000_spi = Spi::spi1(
                    device.SPI1,(dw1000_clk, dw1000_miso, dw1000_mosi),
                    MODE_0,
                    dw1000_spi_freq.into(),
                    clocks
                );
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut dw1000_reset  = gpioc.pc11.into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper); // open drain, do not pull high
                let mut dw1000_cs = gpioa.pa8.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
                let dw1000_clk    = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                let dw1000_mosi   = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                let dw1000_miso   = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                //let _dw1000_wakeup = gpioc.pc5;
                let trace_pin = gpioc.pc8.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
                let mut dw1000_irq = gpioc.pc9.into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);
                let mut dw1000_spi = Spi::spi1(
                    device.SPI1,
                    (dw1000_clk, dw1000_miso, dw1000_mosi),
                    MODE_0,
                    dw1000_spi_freq,
                    clocks,
                    &mut rcc.apb2,
                );
            }
        }
        dw1000_cs.set_high().ok();
        dw1000_reset.set_low().ok();
        busywait!(ms_alt, clocks, 2);
        dw1000_reset.set_high().ok();
        busywait!(ms_alt, clocks, 5);
        let dw1000 = DW1000::new(dw1000_spi, dw1000_cs);
        let mut dw1000 = dw1000.init().unwrap();
        dw1000.set_address(config::PAN_ID, config::UWB_ADDR).unwrap();

        // To VESC
        use hal::serial::{Serial, Event, config::Config};
        let usart1_tx = gpiob.pb6.into_alternate_af7();
        let usart1_rx = gpiob.pb7.into_alternate_af7();
        let mut vesc_serial = Serial::usart1(
            device.USART1,
            (usart1_tx, usart1_rx),
            Config::default().baudrate(115_200.bps()),
            clocks
        ).unwrap();
        vesc_serial.listen(Event::Rxne);
        let vesc_framer = crc_framer::CrcFramerDe::new();
        let (vesc_bbbuffer_p, vesc_bbbuffer_c) = VESC_BBBUFFER.try_split().unwrap();

        // To Ctrl
        let usart2_tx = gpioa.pa2.into_alternate_af7();
        let usart2_rx = gpioa.pa3.into_alternate_af7();
        let mut ctrl_serial = Serial::usart2(
            device.USART2,
            (usart2_tx, usart2_rx),
            Config::default().baudrate(115_200.bps()),
            clocks
        ).unwrap();
        ctrl_serial.listen(Event::Rxne);
        let ctrl_framer = crc_framer::CrcFramerDe::new();
        let (ctrl_bbbuffer_p, ctrl_bbbuffer_c) = CTRL_BBBUFFER.try_split().unwrap();

        let usart6_tx = gpioc.pc6.into_alternate_af8();
        let mut lift_serial = Serial::usart6(
            device.USART6,
            (usart6_tx, NoRx),
            Config::default().baudrate(115_200.bps()),
            clocks
        ).unwrap();

        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let mecanum_wheels = MecanumWheels::default();
                let slaves = Slaves::default();
            } else if  #[cfg(feature = "slave")] {
                let wheel = MCData::default();
            }
        }

        rprintln!("init(): done");

        cx.spawn.blinker().unwrap();
        cx.spawn.radio_chrono().ok(); // TODO: error out
        let (radio_commands_p, radio_commands_c) = RADIO_COMMANDS_QUEUE.split();

        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                init::LateResources {
                    clocks,

                    radio_state: RadioState::Ready(Some(dw1000)),
                    radio_irq: dw1000_irq,
                    radio_trace: trace_pin,
                    radio_commands_p, radio_commands_c,

                    vesc_serial,
                    vesc_framer,
                    vesc_bbbuffer_p, vesc_bbbuffer_c,

                    ctrl_serial,
                    ctrl_framer,
                    ctrl_bbbuffer_p, ctrl_bbbuffer_c,

                    lift_serial,

                    mecanum_wheels, slaves,

                    led_blinky,
                    idle_counter: Wrapping(0u32),
                    exti,

                    stat: Stat::default()
                }
            } else if #[cfg(feature = "slave")] {
                init::LateResources {
                    clocks,

                    radio_state: RadioState::Ready(Some(dw1000)),
                    radio_irq: dw1000_irq,
                    radio_trace: trace_pin,
                    radio_commands_p, radio_commands_c,

                    vesc_serial,
                    vesc_framer,
                    vesc_bbbuffer_p, vesc_bbbuffer_c,

                    ctrl_serial,
                    ctrl_framer,
                    ctrl_bbbuffer_p, ctrl_bbbuffer_c,

                    lift_serial,

                    wheel,

                    led_blinky,
                    idle_counter: Wrapping(0u32),
                    exti
                }
            }
        }
    }

    #[idle(resources = [idle_counter, stat, mecanum_wheels])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.resources.idle_counter.lock(|counter| *counter += Wrapping(1u32));
            cortex_m::asm::delay(20_000_000);

            cfg_if::cfg_if! {
                if #[cfg(feature = "master")] {
                    rprintln!(=> 8, "\x1b[2J\x1b[0m");
                    let (tr_gts_answers, bl_gts_answers, br_gts_answers) = cx.resources.stat.lock(|s|
                        (
                            s.tr_gts_answers,
                            s.bl_gts_answers,
                            s.br_gts_answers,
                        )
                    );
                    rprintln!(=> 8, "TR: {}\n", tr_gts_answers);
                    rprintln!(=> 8, "BL: {}\n", bl_gts_answers);
                    rprintln!(=> 8, "BR: {}\n", br_gts_answers);

                    let (tacho_tl, tacho_tr, tacho_bl, tacho_br) = cx.resources.mecanum_wheels.lock(|wheels| {
                        (
                            wheels.top_left.tacho.0 - wheels.top_left.tacho_shift,
                            wheels.top_right.tacho.0 - wheels.top_right.tacho_shift,
                            wheels.bottom_left.tacho.0 - wheels.bottom_left.tacho_shift,
                            wheels.bottom_right.tacho.0 - wheels.bottom_right.tacho_shift
                        )
                    });
                    rprintln!(=> 8, "TL: {}\tTR: {}", tacho_tl, tacho_tr);
                    rprintln!(=> 8, "BL: {}\tBR: {}", tacho_bl, tacho_br)
                }
            }


            //atomic::compiler_fence(Ordering::SeqCst);
        }
    }

    #[task(resources = [led_blinky, &clocks], schedule = [blinker])]
    fn blinker(cx: blinker::Context) {
        static mut LED_STATE: bool = false;
        static mut DOT_COUNTER: u8 = 0;
        rprint!(=> 2, ".");
        *DOT_COUNTER += 1;
        if *DOT_COUNTER == 30 {
            rprintln!(=> 2, "");
            *DOT_COUNTER = 0;
        }
        //cx.resources.p.enqueue(*DOT_COUNTER).unwrap();

        if *LED_STATE {
            cx.resources.led_blinky.set_low().ok();
            *LED_STATE = false;
        } else {
            cx.resources.led_blinky.set_high().ok();
            *LED_STATE = true;
        }

        //cx.spawn.lift_control(LiftControlCommand::AllUp);
        cx.schedule.blinker(cx.scheduled + ms2cycles!(cx, config::BLINK_PERIOD_MS)).unwrap();
    }

    /// Master states:
    /// 1. pwr_on -> each GTS (guaranteed time slot) send `multicast noack info` to slaves.
    /// `multicast noack info`: RPM, time slot location and duration, time?
    /// 2. Listen for replies from each slave.
    /// 3. Start from 1.
    ///
    /// Slave states:
    /// 1. pwr_on -> listen for GTS packet.
    /// 2. if nothing, send alive packet periodically.
    /// 3. After GTS packet received wait for allocated time slot and send `slave noack info` back.
    /// `slave noack info`: tacho value, power_in, power_motor
    #[task(priority = 5, resources = [&clocks, radio_commands_p, mecanum_wheels], schedule = [radio_chrono])]
    fn radio_chrono(cx: radio_chrono::Context) {
        static mut STATE: RadioChronoState = RadioChronoState::Idle;
        const GTS_END_MS: u32 = 30;
        //use radio::Command;
        //use radio::message::*;
        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let now = DWT::get_cycle_count();

                use crate::radio::message::{GTSEntry, GTSStart, GTSDownlinkData};
                use crate::radio::{Command};
                use RadioChronoState::*;
                *STATE = match STATE {
                    Idle => {
                        let wheels = cx.resources.mecanum_wheels;
                        let tr = GTSEntry::new(0, 6_000, GTSDownlinkData{ rpm: wheels.top_right.rpm.0 });
                        let bl = GTSEntry::new(16_000, 6_000, GTSDownlinkData{ rpm: wheels.bottom_left.rpm.0 });
                        let br = GTSEntry::new(22_000, 6_000, GTSDownlinkData{ rpm: wheels.bottom_right.rpm.0 });
                        let gts_start = GTSStart::new(tr, bl, br);
                        cx.resources.radio_commands_p.enqueue(Command::GTSStart(gts_start)).ok(); // TODO: Count errors
                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, GTS_END_MS)).unwrap();
                        //
                        RadioChronoState::GTSInProgress
                    },
                    GTSInProgress => {
                        cx.resources.radio_commands_p.enqueue(Command::GTSEnd).ok(); // TODO: Count errors
                        cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS - GTS_END_MS)).unwrap();
                        //
                        Idle
                    }
                }
            } else if #[cfg(all(feature = "slave", not(feature = "devnode")))] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).ok(); // TODO: count errors
            } else if #[cfg(feature = "devnode")] {

            }
        }
        rtic::pend(config::DW1000_IRQ_EXTI);
    }

    #[task(binds = EXTI0, priority = 6, resources = [radio_state, radio_irq, radio_commands_c, radio_trace, idle_counter, exti], spawn = [radio_event])]
    fn radio_irq(cx: radio_irq::Context) {
        static mut RX_BUFFER: [u8; 64] = [0u8; 64];
        static mut LAST_IDLE_COUNTER: Wrapping<u32> = Wrapping(0u32);
        static mut LAST_IDLE_INSTANT: i32 = 0i32;

        let now = DWT::get_cycle_count() as i32;
        let dt = now.wrapping_sub(*LAST_IDLE_INSTANT);
        if dt < 0 {
            *LAST_IDLE_INSTANT = now;
        } else if dt > 72_000_000 * 2 {
            if *LAST_IDLE_COUNTER == *cx.resources.idle_counter {
                rprintln!("IRQ lockup detected!");
                cx.resources.radio_irq.disable_interrupt(cx.resources.exti);
            }
            *LAST_IDLE_COUNTER = *cx.resources.idle_counter;
            *LAST_IDLE_INSTANT = now;
        }
        //cx.resources.radio_trace.set_high().ok();
        cx.resources.radio_irq.clear_interrupt_pending_bit();

        radio::state_machine::advance(
            cx.resources.radio_state,
            cx.resources.radio_commands_c,
            RX_BUFFER,
            &cx.spawn,
            cx.resources.radio_trace
        );
        for _ in 0..100 {
            if cx.resources.radio_irq.is_high().unwrap() {
                radio::state_machine::advance(
                    cx.resources.radio_state,
                    cx.resources.radio_commands_c,
                    RX_BUFFER,
                    &cx.spawn,
                    cx.resources.radio_trace
                );
            } else {
                break;
            }
        }
        //cx.resources.radio_trace.set_low().ok();
    }

    #[task(
        priority = 4,
        capacity = 16,
        resources = [&clocks, wheel, mecanum_wheels, radio_commands_p, stat],
        spawn = [motor_control, ctrl_link_control],
        schedule = [radio_event]
    )]
    fn radio_event(mut cx: radio_event::Context, e: radio::Event) {
        rprintln!("radio_event: {:?}", e);
        use radio::Event::*;
        match e {
            #[cfg(feature = "master")]
            GTSAnswerReceived(from, answer) => {
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
                cx.resources.radio_commands_p.lock(|commands|
                    commands.enqueue(radio::Command::GTSSendAnswer(tx_time, gts_answer))).ok(); // TODO: Count errors
                // Schedule an rpm sending after all GTS, so that each MC receive the command at the same time
                let rpm = Rpm(gts_entry.sync_no_ack_data.rpm);
                cx.resources.wheel.rpm = rpm;
                cx.schedule.radio_event(cx.scheduled + ms2cycles!(cx, gts_end_dt.0 / 1000), radio::Event::GTSEnded);
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
                        cx.spawn.ctrl_link_control();
                    } else if  #[cfg(feature = "slave")] {
                        let rpm = cx.resources.wheel.rpm;
                    }
                }
                cx.spawn.motor_control(MotorControlEvent::SetRpm(rpm));
                cx.spawn.motor_control(MotorControlEvent::RequestTelemetry);
            }
        }
    }

    #[task(priority = 2, capacity = 4, resources = [mecanum_wheels, ctrl_bbbuffer_p])]
    fn ctrl_link_control(mut cx: ctrl_link_control::Context) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let (tacho_tl, tacho_tr, tacho_bl, tacho_br) = cx.resources.mecanum_wheels.lock(|wheels| {
                    (
                        wheels.top_left.tacho.0 - wheels.top_left.tacho_shift,
                        wheels.top_right.tacho.0 - wheels.top_right.tacho_shift,
                        wheels.bottom_left.tacho.0 - wheels.bottom_left.tacho_shift,
                        wheels.bottom_right.tacho.0 - wheels.bottom_right.tacho_shift
                    )
                });
                let mut tacho_arr_frame = [0u8; 17];
                tacho_arr_frame[0] = VESC_TACHO_ARRAY_FRAME_ID;
                tacho_arr_frame[1..=4].copy_from_slice(&tacho_tl.to_be_bytes());
                tacho_arr_frame[5..=8].copy_from_slice(&tacho_tr.to_be_bytes());
                tacho_arr_frame[9..=12].copy_from_slice(&tacho_bl.to_be_bytes());
                tacho_arr_frame[13..=16].copy_from_slice(&tacho_br.to_be_bytes());
                cx.resources.ctrl_bbbuffer_p.lock(|bb| {
                    crc_framer::CrcFramerSer::commit_frame(&tacho_arr_frame, bb).ok(); // TODO: count errors
                });
                rtic::pend(CTRL_IRQ_EXTI);
            }
        }
    }

    #[task(priority = 2, capacity = 4, resources = [mecanum_wheels, vesc_bbbuffer_p])]
    fn motor_control(mut cx: motor_control::Context, e: MotorControlEvent) {
        static mut STOPPED: bool = false;
        rprintln!("mc_event: {:?}", e);
        use MotorControlEvent::*;
        match e {
            #[cfg(feature = "master")]
            SetRpmArray(rpms) => {
                cx.resources.mecanum_wheels.lock(|wheels| {
                    wheels.top_left.rpm = rpms.top_left;
                    wheels.top_right.rpm = rpms.top_right;
                    wheels.bottom_left.rpm = rpms.bottom_left;
                    wheels.bottom_right.rpm = rpms.bottom_right;
                });
            },
            #[cfg(feature = "master")]
            MovePreserveHeading(xyt) => {

            },
            #[cfg(feature = "master")]
            MoveIgnoreHeading(xyt) => {

            },
            SetRpm(rpm) => {
                if rpm.0 != 0 || !*STOPPED {
                    let mut setrpm_frame = [0u8; 5];
                    setrpm_frame[0] = VESC_SETRPM_FRAME_ID;
                    setrpm_frame[1..=4].copy_from_slice(&rpm.0.to_be_bytes());
                    crc_framer::CrcFramerSer::commit_frame(&setrpm_frame, cx.resources.vesc_bbbuffer_p).ok(); // TODO: count errors
                    rtic::pend(VESC_IRQ_EXTI);
                    if rpm.0 == 0 {
                        *STOPPED = true;
                    } else {
                        *STOPPED = false;
                    }
                }

            },
            RequestTelemetry => {
                let mut request = [0u8; 5];
                request[0] = VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID;
                request[1..=4].copy_from_slice(&VESC_REQUESTED_VALUES.to_be_bytes());
                crc_framer::CrcFramerSer::commit_frame(&request, cx.resources.vesc_bbbuffer_p).ok(); // TODO: count errors
                rtic::pend(VESC_IRQ_EXTI);
            },
            ResetTacho => {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "master")] {
                        cx.resources.mecanum_wheels.lock(|wheels| {
                            wheels.top_left.tacho_shift = wheels.top_left.tacho.0;
                            wheels.top_right.tacho_shift = wheels.top_right.tacho.0;
                            wheels.bottom_left.tacho_shift = wheels.bottom_left.tacho.0;
                            wheels.bottom_right.tacho_shift = wheels.bottom_right.tacho.0;
                        });
                    }
                }
            }
        }
    }

    #[task(binds = USART1, priority = 3, resources = [vesc_serial, vesc_framer, vesc_bbbuffer_c, mecanum_wheels, wheel])]
    fn vesc_serial_irq(mut cx: vesc_serial_irq::Context) {
        static mut SENDING: Option<bbqueue::GrantR<'static, VescBBBufferSize>> = None;
        static mut SENDING_IDX: usize = 0;
        use core::convert::TryInto;
        let serial = cx.resources.vesc_serial;
        if serial.is_rxne() {
            match serial.read() {
                Ok(byte) => {
                    let framer = cx.resources.vesc_framer;
                    //let bbbuffer_p = cx.resources.ctrl_bbbuffer_p;
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "master")] {
                            let mut wheels = cx.resources.mecanum_wheels;
                        } else if  #[cfg(feature = "slave")] {
                            let mut wheel = cx.resources.wheel;
                        }
                    }
                    //rprintln!(=> 5, "{} {:02x}\n", byte, byte);
                    framer.eat_byte(byte, |frame| {
                        //rprintln!(=> 5, "frame vesc: {} {}\n", frame[0], frame.len());
                        if frame[0] == VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID && frame.len() == 15 {
                            let current_in = i32::from_be_bytes(frame[5..=8].try_into().unwrap());
                            let current_in = (current_in as f32) / 100.0f32;
                            let voltage_in = i16::from_be_bytes(frame[9..=10].try_into().unwrap());
                            let voltage_in = voltage_in as f32;
                            let power_in = current_in * voltage_in;
                            let tacho = i32::from_be_bytes(frame[11..=14].try_into().unwrap());
                            let tacho = Tachometer(tacho);
                            //rprintln!(=> 5, "i_in:{} v_in:{} tacho:{}", current_in, voltage_in, tacho.0);

                            cfg_if::cfg_if! {
                                if #[cfg(feature = "master")] {
                                    wheels.lock(|wheels| {
                                        wheels.top_left.tacho = tacho;
                                        wheels.top_left.power_in = power_in;
                                    });
                                } else if  #[cfg(feature = "slave")] {
                                    wheel.lock(|wheel| {
                                        wheel.tacho = tacho;
                                        wheel.power_in = power_in;
                                    });
                                }
                            }
                        }
                    });
                },
                _ => {}
            }
        }

        use hal::serial::Event;
        //let _:() = cx.resources.vesc_bbbuffer_c.read().unwrap();
        *SENDING = if SENDING.is_some() {
            let rgr = SENDING.take().unwrap();
            let already_sent = *SENDING_IDX;
            if already_sent == rgr.len() {
                rgr.release(already_sent); // Previous block is finished
                serial.unlisten(Event::Txe);
                None // Try send more!
            } else {
                let next = rgr[already_sent];
                match serial.write(next) {
                    Ok(_) => {
                        *SENDING_IDX += 1;
                        Some(rgr)
                    },
                    _ => {
                        Some(rgr)
                    }
                }
            }
        } else {
            match cx.resources.vesc_bbbuffer_c.read() {
                Ok(rgr) => {
                    let next = rgr[0];
                    serial.listen(Event::Txe);
                    match serial.write(next) {
                        Ok(_) => {
                            *SENDING_IDX = 1;
                            Some(rgr)
                        },
                        _ => {
                            *SENDING_IDX = 0;
                            Some(rgr)
                        }
                    }
                },
                Err(_) => {
                    None
                }
            }
        }
    }

    #[task(binds = USART2, priority = 3, resources = [ctrl_serial, ctrl_framer, ctrl_bbbuffer_p, ctrl_bbbuffer_c], spawn = [motor_control, lift_control])]
    fn ctrl_serial_irq(cx: ctrl_serial_irq::Context) {
        static mut SENDING: Option<bbqueue::GrantR<'static, CtrlBBBufferSize>> = None;
        static mut SENDING_IDX: usize = 0;

        use core::convert::TryInto;
        let serial = cx.resources.ctrl_serial;

        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                match serial.read() {
                    Ok(byte) => {
                        let framer = cx.resources.ctrl_framer;
                        //let bbbuffer_p = cx.resources.vesc_bbbuffer_p;
                        let spawner = cx.spawn;
                        framer.eat_byte(byte, |frame| {
                            rprintln!(=> 5, "frame ctrl: {} {}\n", frame[0], frame.len());
                            if frame[0] == VESC_RPM_ARRAY_FRAME_ID {
                                let tl_rpm = Rpm( i32::from_be_bytes(frame[1..=4].try_into().unwrap()) );
                                let tr_rpm = Rpm( i32::from_be_bytes(frame[5..=8].try_into().unwrap()) );
                                let bl_rpm = Rpm( i32::from_be_bytes(frame[9..=12].try_into().unwrap()) );
                                let br_rpm = Rpm( i32::from_be_bytes(frame[13..=16].try_into().unwrap()) );
                                let rpm_array = RpmArray {
                                    top_left: tl_rpm,
                                    top_right: tr_rpm,
                                    bottom_left: bl_rpm,
                                    bottom_right: br_rpm
                                };
                                spawner.motor_control(MotorControlEvent::SetRpmArray(rpm_array));
                                //rprintln!("{} {} {} {}", tl_rpm, tr_rpm, bl_rpm, br_rpm);
                            } else if frame[0] == VESC_LIFT_FRAME_ID && frame.len() == 2 {
                                rprintln!(=>5, "lift cmd: {}", frame[1] as char);
                                use LiftControlCommand::*;
                                match frame[1] as char {
                                    'q' => { spawner.lift_control(LeftDown).ok(); },
                                    'a' => { spawner.lift_control(LeftUp).ok(); },
                                    'w' => { spawner.lift_control(AllDown).ok(); },
                                    's' => { spawner.lift_control(AllUp).ok(); },
                                    'e' => { spawner.lift_control(RightDown).ok(); },
                                    'd' => { spawner.lift_control(RightUp).ok(); },
                                    _ => {}
                                };
                            } else if frame[0] == VESC_RESET_ALL {
                                rprintln!(=>5, "\n\nRESET TACHO\n\n");
                                spawner.motor_control(MotorControlEvent::ResetTacho);
                            }
                            //crc_framer::CrcFramerSer::commit_frame(frame, bbbuffer_p);
                            //rtic::pend(VESC_IRQ_EXTI);
                        });
                    },
                    _ => {}
                }
            }
        }


        use hal::serial::Event;
        //let _:() = cx.resources.vesc_bbbuffer_c.read().unwrap();
        *SENDING = if SENDING.is_some() {
            let rgr = SENDING.take().unwrap();
            let already_sent = *SENDING_IDX;
            if already_sent == rgr.len() {
                rgr.release(already_sent); // Previous block is finished
                serial.unlisten(Event::Txe);
                None // Try send more!
            } else {
                let next = rgr[already_sent];
                match serial.write(next) {
                    Ok(_) => {
                        *SENDING_IDX += 1;
                        Some(rgr)
                    },
                    _ => {
                        Some(rgr)
                    }
                }
            }
        } else {
            match cx.resources.ctrl_bbbuffer_c.read() {
                Ok(rgr) => {
                    let next = rgr[0];
                    serial.listen(Event::Txe);
                    match serial.write(next) {
                        Ok(_) => {
                            *SENDING_IDX = 1;
                            Some(rgr)
                        },
                        _ => {
                            *SENDING_IDX = 0;
                            Some(rgr)
                        }
                    }
                },
                Err(_) => {
                    None
                }
            }
        }
    }

    #[task(priority = 2, capacity = 4, resources = [&clocks, lift_serial], schedule = [lift_control])]
    fn lift_control(cx: lift_control::Context, command: LiftControlCommand) {
        static mut TOKENS: u16 = 0;
        static mut CURRENT_SYMBOL: u8 = 0;
        const RESEND_TIME_MS: u32 = 10;
        const ABORT_TIME_MS: u32 = 250;
        const ALL_UP_LONG_TIME: u32 = 25_000;
        const ALL_DOWN_LONG_TIME: u32 = 25_000;

        const ABORT_TOKENS: u16 = (ABORT_TIME_MS / RESEND_TIME_MS) as u16;
        const ALL_UP_LONG_TOKENS: u16 = (ALL_UP_LONG_TIME / RESEND_TIME_MS) as u16;
        const ALL_DOWN_LONG_TOKENS: u16 = (ALL_DOWN_LONG_TIME / RESEND_TIME_MS) as u16;

        rprintln!(=> 6, "{:?} {}", command, *TOKENS);
        use LiftControlCommand::*;
        if let Continue = command {
            if *TOKENS != 0 {
                cx.resources.lift_serial.write(*CURRENT_SYMBOL).ok();
                cx.schedule.lift_control(cx.scheduled + ms2cycles!(cx, 10), Continue).ok();
                *TOKENS -= 1;
            }
        } else if let Stop = command {
            *TOKENS = 0;
        } else {
            let prev_tokens = *TOKENS;
            match command {
                LeftUp => {      *CURRENT_SYMBOL = 'a' as u8; *TOKENS = ABORT_TOKENS; },
                LeftDown => {    *CURRENT_SYMBOL = 'q' as u8; *TOKENS = ABORT_TOKENS; },
                RightUp => {     *CURRENT_SYMBOL = 'd' as u8; *TOKENS = ABORT_TOKENS; },
                RightDown => {   *CURRENT_SYMBOL = 'e' as u8; *TOKENS = ABORT_TOKENS; },
                AllUp => {       *CURRENT_SYMBOL = 's' as u8; *TOKENS = ABORT_TOKENS; },
                AllDown => {     *CURRENT_SYMBOL = 'w' as u8; *TOKENS = ABORT_TOKENS; },
                AllUpLong => {   *CURRENT_SYMBOL = 's' as u8; *TOKENS = ALL_UP_LONG_TOKENS; },
                AllDownLong => { *CURRENT_SYMBOL = 'w' as u8; *TOKENS = ALL_DOWN_LONG_TOKENS; },
                _ => { unreachable!() },
            }
            if prev_tokens == 0 {
                cx.resources.lift_serial.write(*CURRENT_SYMBOL).ok();
                cx.schedule.lift_control(cx.scheduled + ms2cycles!(cx, 10), Continue).ok();
            }
        }
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
        fn EXTI9_5();
    }
};

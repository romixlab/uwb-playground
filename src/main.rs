#![no_main]
#![no_std]

mod board;
mod panic_handler;
mod radio;
#[macro_use]
mod util;
mod config;

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


#[app(device = crate::board::hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,

        radio_state: RadioState,
        radio_irq: RadioIrqPin,
        radio_trace: RadioTracePin,
        radio_commands_p: radio::CommandQueueP,
        radio_commands_c: radio::CommandQueueC,

        led_blinky: LedBlinkyPin,

        idle_counter: Wrapping<u32>,
        exti: hal::stm32::EXTI,
    }

    #[init(schedule = [], spawn = [radio_chrono, blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RADIO_COMMANDS_QUEUE: radio::CommandQueue = heapless::spsc::Queue(heapless::i::Queue::new());

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
        dw1000.set_address(mac::PanId(0x0d57), mac::ShortAddress(0xaabb)).unwrap();

        rprintln!("init(): done");

        cx.spawn.blinker().unwrap();
        cx.spawn.radio_chrono().unwrap();
        let (radio_commands_p, radio_commands_c) = RADIO_COMMANDS_QUEUE.split();

        init::LateResources {
            clocks,

            radio_state: RadioState::Ready(Some(dw1000)),
            radio_irq: dw1000_irq,
            radio_trace: trace_pin,
            radio_commands_p, radio_commands_c,

            led_blinky,
            idle_counter: Wrapping(0u32),
            exti
        }
    }

    #[idle(resources = [idle_counter])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.resources.idle_counter.lock(|counter| *counter += Wrapping(1u32));
            cortex_m::asm::delay(20_000_000);
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
    #[task(priority = 2, resources = [&clocks, radio_commands_p], schedule = [radio_chrono])]
    fn radio_chrono(cx: radio_chrono::Context) {
        //use radio::Command;
        //use radio::message::*;
        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let now = DWT::get_cycle_count();

                use crate::radio::message::{GTSEntry, GTSStart, GTSDownlinkData};
                use crate::radio::{Command};
                let tr = GTSEntry::new(0, 1_000, GTSDownlinkData{ rpm: now });
                let bl = GTSEntry::new(16_000, 2_000, GTSDownlinkData{ rpm: now + 1 });
                let br = GTSEntry::new(22_000, 3_000, GTSDownlinkData{ rpm: now + 2 });
                let gts_start = GTSStart::new(tr, bl, br);
                let _result = cx.resources.radio_commands_p.enqueue(Command::GTSStart(gts_start));
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS)).unwrap();
            } else if #[cfg(feature = "slave")] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).unwrap();
            }
        }
        rtic::pend(config::DW1000_IRQ_EXTI);
    }

    #[task(binds = EXTI0, resources = [radio_state, radio_irq, radio_commands_c, radio_trace, idle_counter, exti], priority = 3)]
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
            cx.resources.radio_trace
        );
        for _ in 0..100 {
            if cx.resources.radio_irq.is_high().unwrap() {
                radio::state_machine::advance(
                    cx.resources.radio_state,
                    cx.resources.radio_commands_c,
                    RX_BUFFER,
                    cx.resources.radio_trace
                );
            } else {
                break;
            }
        }
        //cx.resources.radio_trace.set_low().ok();
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
    }
};

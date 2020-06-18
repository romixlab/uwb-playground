#![no_main]
#![no_std]

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
    UWBRadio,
    RadioState
};

use core::sync::atomic::{self, Ordering};
use rtic::app;
use rtic::cyccnt::U32Ext;
use stm32f4xx_hal as hal;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::gpio::{PushPull, Output, Input, Edge, ExtiPin, PullDown};
use stm32f4xx_hal::gpio::{gpioa::{PA0, PA1}, gpiob::{PB4, PB5, PB8, PB9}};
use rtt_target::{rtt_init_print, rprintln, rprint};
use cortex_m::peripheral::DWT;
// use heapless::{
//     consts::U4,
//     i,
//     spsc::{Consumer, Producer, Queue}
// };
use hal::{
    spi::Spi,
};
use embedded_hal::spi::MODE_0;

use embedded_hal::digital::v2::OutputPin;
use stm32f4xx_hal::rcc::Clocks;


#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,
        uwb: UWBRadio,
        uwb_irq: PA0<Input<PullDown>>,
        #[init(RadioState::Idle)]
        radio_state: RadioState,
        led1_red: PB4<Output<PushPull>>,
        led1_green: PB5<Output<PushPull>>,
        led2_red: PB8<Output<PushPull>>,
        led2_green: PB9<Output<PushPull>>,
        gpio0: PA1<Output<PushPull>>,
        //gpio1: PA0<Input<PullDown>>,
        //p: Producer<'static, u8, U4>,
        //c: Consumer<'static, u8, U4>
    }

    #[init(schedule = [], spawn = [radio_chrono, blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        //static mut Q: Queue<u8, U4> = Queue(i::Queue::new());

        rtt_init_print!(NoBlockSkip);
        rprintln!("\x1b[2J\x1b[0m");
        rprintln!("init()");

        let mut core/*: cortex_m::Peripherals */= cx.core;
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let device: stm32f4xx_hal::stm32::Peripherals = cx.device;
        // device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
        //let _flash = device.FLASH;
        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        let mut syscfg = device.SYSCFG;
        let mut exti = device.EXTI;

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();

        let mut led1_red = gpiob.pb4.into_push_pull_output();
        let mut led1_green = gpiob.pb5.into_push_pull_output();
        let mut led2_red = gpiob.pb8.into_push_pull_output();
        let mut led2_green = gpiob.pb9.into_push_pull_output();
        led1_green.set_low().ok();
        led1_red.set_low().ok();
        led2_green.set_high().ok();
        led2_red.set_low().ok();

        let gpio0 = gpioa.pa1.into_push_pull_output();
        let mut gpio1 = gpioa.pa0.into_pull_down_input();
        gpio1.make_interrupt_source(&mut syscfg);
        gpio1.trigger_on_edge(&mut exti, Edge::RISING);
        gpio1.enable_interrupt(&mut exti);

        // DW1000
        let mut dw1000_reset  = gpiob.pb0.into_open_drain_output(); // open drain, do not pull high
        let mut dw1000_cs = gpioa.pa4.into_push_pull_output();
        dw1000_cs.set_high().ok();
        let dw1000_clk    = gpioa.pa5.into_alternate_af5();
        let dw1000_mosi   = gpioa.pa7.into_alternate_af5();
        let dw1000_miso   = gpioa.pa6.into_alternate_af5();
        let _dw1000_wakeup = gpioc.pc5;
        //let mut dw1000_irq    = gpioc.pc4.into_pull_down_input();
        let dw1000_spi_freq = 1.mhz();
        let dw1000_spi = Spi::spi1(
            device.SPI1,(dw1000_clk, dw1000_miso, dw1000_mosi),
            MODE_0,
            dw1000_spi_freq.into(),
            clocks);
        dw1000_reset.set_low().ok();
        busywait!(ms_alt, clocks, 2);
        dw1000_reset.set_high().ok();
        busywait!(ms_alt, clocks, 5);

        let dw1000 = DW1000::new(dw1000_spi, dw1000_cs);
        //dw1000_irq.make_interrupt_source(&mut syscfg);
        //dw1000_irq.trigger_on_edge(&mut exti, Edge::RISING);
        //dw1000_irq.enable_interrupt(&mut exti);
        let mut dw1000 = dw1000.init().unwrap();
        dw1000.set_address(mac::PanId(0x0d57), mac::ShortAddress(0xaabb)).unwrap();

        //let (p, c) = Q.split();
        rprintln!("init(): done {}", clocks.sysclk().0 / 1000 * 2);

        cx.spawn.blinker().unwrap();
        cx.spawn.radio_chrono().unwrap();

        init::LateResources {
            clocks,
            uwb: UWBRadio::Ready(Some(dw1000)),
            uwb_irq: gpio1,
            gpio0,
            led1_green, led1_red, led2_green, led2_red,
            //p, c
        }
    }

    #[idle(resources = [led2_red])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            //if let Some(_) = cx.resources.c.dequeue() {
                //rprintln!("dequeue: {}", byte);
            //}
            //cx.resources.gpio1.set_high().ok();
            //delay(100);
            //cx.resources.gpio1.set_low().ok();
            //delay(100);
            atomic::compiler_fence(Ordering::SeqCst);
        }
    }

    #[task(resources = [led1_green, &clocks], schedule = [blinker])]
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
            cx.resources.led1_green.set_low().ok();
            *LED_STATE = false;
        } else {
            cx.resources.led1_green.set_high().ok();
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
    #[task(priority = 2, resources = [&clocks, radio_state], schedule = [radio_chrono])]
    fn radio_chrono(mut cx: radio_chrono::Context) {
        use RadioState::*;
        cx.resources.radio_state.lock(|state| {
            if *state == Idle {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "master")] {
                        *state = GTSStart;
                    } else if #[cfg(feature = "slave")] {
                        *state = GTSStartWait;
                    }
                }
            }
        });
        cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::GTS_PERIOD_MS)).unwrap();
            } else if #[cfg(feature = "slave")] {
                cx.schedule.radio_chrono(cx.scheduled + ms2cycles!(cx, config::DW1000_CHECK_PERIOD_MS)).unwrap();
            }
        }
        rtic::pend(config::DW1000_IRQ_EXTI);
    }

    #[task(binds = EXTI0, resources = [uwb, uwb_irq, radio_state, gpio0], priority = 3)]
    fn radio_irq(cx: radio_irq::Context) {
        static mut RX_BUFFER: [u8; 64] = [0u8; 64];
        cx.resources.uwb_irq.clear_interrupt_pending_bit();

        radio::state_machine::on_radio_event(
            cx.resources.uwb,
            cx.resources.radio_state,
            RX_BUFFER,
            cx.resources.gpio0
        );
    }

    extern "C" {
        fn EXTI2();
        fn EXTI1();
    }
};

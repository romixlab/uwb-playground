#![no_main]
#![no_std]

mod board;
mod panic_handler;
mod color;

use cortex_m_rt as rt;
use board::hal::prelude::*;
use board::hal::stm32;
use rt::entry;
use rtic::app;
use stm32g4xx_hal::rcc::Config;
use stm32g4xx_hal::cortex_m;
use board::hal::timer::Timer;
use board::hal::gpio::{Output, PushPull};
use board::hal::gpio::gpiob::PB15;
use crate::board::hal::interrupt::TIM7;
use cortex_m::peripheral::DWT;
use rtt_target::rprintln;

#[app(device = crate::board::hal::stm32, peripherals = true, monotonic = rtic::cycnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: PB15<Output<PushPull>>,
        timer: Timer<stm32::TIM7>
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let rtt_channels = rtt_target::rtt_init! {
            up: {
                0: { // channel number
                    size: 1024 // buffer size in bytes
                    mode: NoBlockSkip // mode (optional, default: NoBlockSkip, see enum ChannelMode)
                    name: "Terminal" // name (optional, default: no name)
                }
            }
            down: {
                0: {
                    size: 128
                    name: "Terminal"
                }
            }
        };
        rtt_target::set_print_channel(rtt_channels.up.0);
        rprintln!("Genius Charger v{}", env!("CARGO_PKG_VERSION"));

        let mut cp = cx.core;
        cp.DCB.enable_trace();
        DWT::unlock();
        cp.DWT.enable_cycle_counter();

        let dp: stm32::Peripherals = cx.device;

        let mut rcc = dp.RCC.freeze(Config::pll());
        let mut delay = cp.SYST.delay(&rcc.clocks);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let mut led_green = gpiob.pb15.into_push_pull_output();
        let mut led_red = gpioc.pc6.into_push_pull_output();
        led_red.set_high().ok();

        let mut timer = dp.TIM7.timer(&mut rcc);
        timer.start(5.hz());
        timer.listen();

        crate::init::LateResources {
            led: led_green,
            timer
        }
    }

    #[task(binds = TIM7, resources = [timer, led])]
    fn on_timer_tick(cx: on_timer_tick::Context) {
        cx.resources.led.toggle().unwrap();
        cx.resources.timer.clear_irq();

        rprintln!("toggle");
    }
};
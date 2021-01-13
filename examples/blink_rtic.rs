#![no_main]
#![no_std]

use hal::cortex_m;
use cortex_m_rt as rt;
use stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use rt::entry;

use core::panic::PanicInfo;
use hal::rcc::Config;

use rtic::app;
use hal::gpio::gpiob::PB15;
use hal::gpio::{Output, PushPull};
use cortex_m::peripheral::DWT;
use stm32g4xx_hal::rcc::Clocks;
use rtic::cyccnt::U32Ext;
use core::sync::atomic;
use core::sync::atomic::Ordering;
use rtt_target::rprintln;
use stm32g4xx_hal::gpio::{AltFunction, Speed};

#[app(device = stm32g4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,
        status_led: PB15<Output<PushPull>>
    }

    #[init(
        spawn = [blinker]
    )]
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
        rprintln!("RTIC blinker");

        let mut core/*: cortex_m::Peripherals */= cx.core;
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let device: hal::stm32::Peripherals = cx.device;

        let rcc = device.RCC.constrain();
        let mut rcc = rcc.freeze(stm32g4xx_hal::rcc::Config::default());
        let clocks = rcc.clocks;

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);
        let mut led_green = gpiob.pb15.into_push_pull_output();
        let mut led_red = gpioc.pc6.into_push_pull_output();

        // Enable MCO, disable gate drivers first, as MCO is one of the gate signals
        let mut lgate_drv_en = gpioc.pc2.into_push_pull_output();
        let mut rgate_drv_en = gpioc.pc5.into_push_pull_output();
        lgate_drv_en.set_low().ok();
        rgate_drv_en.set_low().ok();
        unsafe {
            let rcc = unsafe { &(*hal::stm32::RCC::ptr()) };
            const MCO_DISABLED: u8       = 0b0000;
            const MCO_SYSCLK: u8         = 0b0001;
            const MCO_HSI16: u8          = 0b0011;
            const MCO_HSE: u8            = 0b0100;
            const MCO_MAIN_PLL: u8       = 0b0101;
            const MCO_LSI: u8            = 0b0110;
            const MCO_INTERNAL_HSI48: u8 = 0b1000;

            const MCO_DIV1: u8  = 0b000;
            const MCO_DIV2: u8  = 0b001;
            const MCO_DIV4: u8  = 0b010;
            const MCO_DIV8: u8  = 0b011;
            const MCO_DIV16: u8 = 0b100;

            rcc.cfgr.modify(|_, w|
                w
                    .mcopre().bits(MCO_DIV16)
                    .mcosel().bits(MCO_SYSCLK)
            );
        };
        let mut mco = gpioa.pa8.into_push_pull_output();
        let mut mco = mco.set_speed(Speed::VeryHigh);
        mco.set_alt_mode(AltFunction::AF0);

        led_red.set_high().ok();

        cx.spawn.blinker().ok();

        init::LateResources {
            clocks,
            status_led: led_green
        }
    }

    #[task(
        resources = [
            &clocks,
            status_led,
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context) {
        cx.resources.status_led.toggle().ok();
        rprintln!("scheduled: {:?}", cx.scheduled);
        cx.schedule.blinker(cx.scheduled + 160_000_000.cycles()).ok();
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {
            atomic::compiler_fence(Ordering::SeqCst);
        }
    }

    extern "C" {
        fn EXTI1();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}
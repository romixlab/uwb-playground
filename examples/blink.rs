#![no_main]
#![no_std]

use hal::cortex_m;
use cortex_m_rt as rt;
use stm32g4xx_hal as hal;

use crate::hal::prelude::*;
use crate::hal::stm32;
use rt::entry;

use core::panic::PanicInfo;
use hal::rcc::Config;

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}

#[entry]
fn main() -> ! {
    let dp = hal::stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.freeze(Config::pll());
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led_green = gpiob.pb15.into_push_pull_output();
    let mut led_red = gpioc.pc6.into_push_pull_output();

    loop {
        led_red.toggle().ok();
        delay.delay(500.ms());
        led_green.toggle().ok();
        delay.delay(500.ms());
    }
}
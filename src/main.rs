#![no_main]
#![no_std]

mod board;
mod panic_handler;
mod color;

use cortex_m_rt as rt;
use board::hal::prelude::*;
use board::hal::stm32;
use rt::entry;
use stm32g4xx_hal::rcc::Config;
use stm32g4xx_hal::cortex_m;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
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
    }
}
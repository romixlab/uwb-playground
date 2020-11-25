use core::panic::PanicInfo;
use rtt_target::rprintln;
use crate::board::hal::cortex_m;
use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use crate::color;
use crate::board::hal::stm32::Peripherals;
use crate::board::hal::gpio::GpioExt;

fn blink_code_impl<O: OutputPin>(mut led: O, delay_cycles: u32) {
    for _ in 0..20 {
        led.set_high().ok();
        delay(delay_cycles);
        led.set_low().ok();
        delay(delay_cycles);
    }
}

#[cfg(feature = "pozyx-board")]
fn blink_led_angrily() {
    let device = unsafe { Peripherals::steal() };
    let gpiob = device.GPIOB.split();
    let led1_red = gpiob.pb4.into_push_pull_output();
    //let mut led2_red = gpiob.pb8.into_push_pull_output();
    blink_code_impl(led1_red, 72_000_000 / 1_000 * 50);
}

#[cfg(feature = "gcharger-board")]
fn blink_led_angrily() {
    use crate::board::hal::rcc::RccExt;

    let device = unsafe { Peripherals::steal() };
    let mut rcc = device.RCC.constrain();
    let gpioc = device.GPIOC.split(&mut rcc);
    let led_red = gpioc.pc6.into_push_pull_output();
    blink_code_impl(led_red, 160_000_000 / 1_000 * 50);
}

#[cfg(feature = "gcarrier-board")]
fn blink_led_angrily() {
    use crate::board::hal::rcc::RccExt;

    let device = unsafe { Peripherals::steal() };
    let mut rcc = device.RCC.constrain();
    let gpiob = device.GPIOB.split(&mut rcc);
    let led_red = gpiob.pb0.into_push_pull_output();
    blink_code_impl(led_red, 160_000_000 / 1_000 * 50);
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!(=>0, "{}!!!!!!!", color::RED);
    rprintln!(=>0, "PANIC: {}", info);
    rprintln!(=>0, "!!!!!!!");
    blink_led_angrily();
    cortex_m::peripheral::SCB::sys_reset(); // -> !
    loop {
        blink_led_angrily();
        use core::sync::atomic::{self, Ordering};
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
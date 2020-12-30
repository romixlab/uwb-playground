use crate::board::hal::cortex_m;
use crate::board::hal::gpio::GpioExt;
use crate::board::hal::stm32::Peripherals;
use crate::color;
use core::panic::PanicInfo;
use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use rtt_target::rprintln;

fn blink_code_impl<O: OutputPin>(mut led: O, delay_cycles: u32) {
    for _ in 0..20 {
        led.set_high().ok();
        delay(delay_cycles);
        led.set_low().ok();
        delay(delay_cycles);
    }
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

use core::panic::PanicInfo;
use rtt_target::{rprintln};
use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use crate::color;

#[cfg(feature = "pozyx-board")]
fn blink_led_angrily() {
    use crate::board::hal::stm32::Peripherals;
    use crate::board::hal::gpio::GpioExt;

    let device = unsafe { Peripherals::steal() };
    let gpiob = device.GPIOB.split();
    let mut led1_red = gpiob.pb4.into_push_pull_output();
    let mut led2_red = gpiob.pb8.into_push_pull_output();

    for _ in 0..20 {
        led1_red.set_high().ok();
        led2_red.set_high().ok();
        delay(72_000_000 / 1_000 * 50);
        led1_red.set_low().ok();
        led2_red.set_low().ok();
        delay(72_000_000 / 1_000 * 50);
    }
}

#[cfg(feature = "dragonfly-board")]
fn blink_led_angrily() {
    use crate::board::hal::stm32::Peripherals;
    use crate::board::hal::gpio::GpioExt;
    use stm32l4xx_hal::rcc::RccExt;

    let device = unsafe { Peripherals::steal() };
    let mut rcc = device.RCC.constrain();
    let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
    let mut led_red = gpiob.pb2.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    for _ in 0..20 {
        led_red.set_high().ok();
        delay(72_000_000 / 1_000 * 50);
        led_red.set_low().ok();
        delay(72_000_000 / 1_000 * 50);
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!(=>6, "{}!!!!!!!", color::RED);
    rprintln!(=>6, "PANIC: {}", info);
    rprintln!(=>6, "!!!!!!!");
    blink_led_angrily();
    cortex_m::peripheral::SCB::sys_reset(); // -> !
    // loop {
    //     use core::sync::atomic::{self, Ordering};
    //     atomic::compiler_fence(Ordering::SeqCst);
    // }
}
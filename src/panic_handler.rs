use core::panic::PanicInfo;
use rtt_target::{rprintln};

fn blink_led_angrily() {
    use stm32f4xx_hal::stm32::Peripherals;
    use stm32f4xx_hal::gpio::GpioExt;
    use cortex_m::asm::delay;
    use embedded_hal::digital::v2::OutputPin;

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

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("\x1b[1;31;40mPANIC: {}", info);
    blink_led_angrily();
    cortex_m::peripheral::SCB::sys_reset(); // -> !
    // loop {
    //     use core::sync::atomic::{self, Ordering};
    //     atomic::compiler_fence(Ordering::SeqCst);
    // }
}
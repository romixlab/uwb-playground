#![deny(unsafe_code)]
#![no_main]
#![no_std]

//use panic_halt as _;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    //rtt_init_print!();
    rprintln!("\x1b[1;31;40mPANIC: {}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

use rtt_target::{rtt_init_print, rprintln, rprint};

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{prelude::*, stm32,
                 spi::{Pins, Spi},
                 timer::Timer
};
use embedded_hal::spi::MODE_0;

use nb::block;
use embedded_timeout_macros::block_timeout;
use dw1000::{
    DW1000,
    mac,
    configs::{
        RxConfig,
        TxConfig,
        SfdSequence,
        BitRate,
        PreambleLength
    }
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    #[cfg(feature = "master")] {
        rprintln!("UWB TX");
    }
    #[cfg(feature = "slave")] {
        rprintln!("UWB RX");
    }

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut led1_red = gpiob.pb4.into_push_pull_output();
    let mut led1_green = gpiob.pb5.into_push_pull_output();
    let mut led2_red = gpiob.pb8.into_push_pull_output();
    let mut led2_green = gpiob.pb9.into_push_pull_output();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // DW1000
    let mut dw1000_reset  = gpiob.pb0.into_open_drain_output(); // open drain, do not pull high
    let mut dw1000_cs = gpioa.pa4.into_push_pull_output();
    dw1000_cs.set_high().ok();
    let dw1000_clk    = gpioa.pa5.into_alternate_af5();
    let dw1000_mosi   = gpioa.pa7.into_alternate_af5();
    let dw1000_miso   = gpioa.pa6.into_alternate_af5();
    let dw1000_wakeup = gpioc.pc5;
    let dw1000_irq    = gpioc.pc4;
    let dw1000_spi_freq = 1.mhz();
    let mut dw1000_spi = Spi::spi1(
        dp.SPI1,(dw1000_clk, dw1000_miso, dw1000_mosi),
        MODE_0,
        dw1000_spi_freq.into(),
        clocks);
    let mut dw1000 = DW1000::new(dw1000_spi, dw1000_cs);

    dw1000_reset.set_low();
    delay.delay_ms(2_u32);
    dw1000_reset.set_high();
    delay.delay_ms(5_u32);
    let mut dw1000 = dw1000.init().unwrap();
    dw1000.set_address(mac::PanId(0x0d57), mac::ShortAddress(0xaabb)).unwrap();
    let mut dw1000_timer = Timer::tim9(dp.TIM9, 1.mhz(), clocks);

    let bitrate = BitRate::Kbps110;
    let preamble = PreambleLength::Symbols2048;
    let sfd_sequence = SfdSequence::Decawave;

    let mut alive_indicator = 0i8;
    loop {
        #[cfg(feature = "master")] {
            let tx_config = TxConfig {
                bitrate,
                preamble_length: preamble,
                sfd_sequence,
                ..TxConfig::default()
            };
            let mut sending = dw1000.send(b"ping", mac::Address::broadcast(&mac::AddressMode::Short), None, tx_config).unwrap();
            let result = block!(sending.wait());
            rprintln!("tx:{:?}", result);
            dw1000 = sending.finish_sending().unwrap();
            led1_green.set_high().unwrap();
            delay.delay_ms(25_u32);
            led1_green.set_low().unwrap();
            delay.delay_ms(25_u32);
        }
        #[cfg(feature = "slave")] {
            let rx_config = RxConfig {
                bitrate,
                expected_preamble_length: preamble,
                sfd_sequence,
                frame_filtering: false,
                ..RxConfig::default()
            };
            let mut receiving = dw1000.receive(rx_config).unwrap();
            let mut buffer = [0; 64];
            dw1000_timer.start(10.hz()); //1ms?
            let result = block_timeout!(&mut dw1000_timer, receiving.wait(&mut buffer));
            dw1000 = receiving.finish_receiving().unwrap();
            match result {
                Ok(m) => {
                    let frame = m.frame;
                    rprintln!("dest:{:?}\tsource:{:?}\tseq:{}\tdata:{:?}", frame.header.destination, frame.header.source, frame.header.seq, frame.payload);
                    led1_green.set_high().unwrap();
                },
                Err(e) => {
                    rprintln!("rxE: {:?}", e);
                    led1_red.set_high().unwrap();
                }
            }
            delay.delay_ms(25_u32);
            led1_green.set_low().unwrap();
            led1_red.set_low().unwrap();
        }

        if alive_indicator >= 0 {
            rprint!(".");
            alive_indicator = alive_indicator + 1;
            if alive_indicator == 30 {
                rprintln!();
                alive_indicator = 0;
            }
        }
    }
}

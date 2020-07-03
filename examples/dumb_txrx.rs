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
        PreambleLength,
        PulseRepetitionFrequency,
        MaximumFrameLength
    }
};

fn check_saw(bytes: &[u8]) -> bool {
    use core::num::Wrapping;
    let mut counter = Wrapping(0u8);
    for b in bytes {
        if *b != counter.0 {
            return false;
        }
        counter += Wrapping(1);
    }
    true
}

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
    let clocks = rcc.cfgr.sysclk(24.mhz()).freeze();

    rprintln!("sysclk={} hclk={} pclk1={} pclk2={}", clocks.sysclk().0, clocks.hclk().0, clocks.pclk1().0, clocks.pclk2().0, );



    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // DW1000
    let mut trace_pin = gpioa.pa1.into_push_pull_output(); // Header pin 1
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
    delay.delay_ms(20_u32);
    dw1000_reset.set_high();
    delay.delay_ms(70_u32);
    let mut dw1000 = dw1000.init(MaximumFrameLength::Decawave1023).unwrap();

    let dw1000_spi_freq_hi = 2.mhz();
    for _ in 0..10 {
        let sys_status = dw1000.ll().sys_status().read().unwrap();
        if sys_status.clkpll_ll() == 0b0 {
            rprint!("SPI speed bump to: {}MHz, ", dw1000_spi_freq_hi.0);
            dw1000.ll().access_spi(|spi| {
                let (old_spi, pins) = spi.free();
                Spi::spi1(
                    old_spi, pins,
                    MODE_0,
                    dw1000_spi_freq_hi.into(),
                    clocks
                )
            });
            let br = unsafe {
                let spi1 = unsafe { &(*hal::stm32::SPI1::ptr()) };
                spi1.cr1.read().br().bits()
            };
            let ratio = 1 << (br + 1);
            rprintln!("done, actual ratio:{} baud:{}", ratio, clocks.pclk2().0 / ratio);
            break;
        } else {
            rprintln!("clkpll_ll = 1, resetting");

            delay.delay_ms(200_u32);
        }
    }
    dw1000.set_address(mac::PanId(0x4321), mac::ShortAddress(0x9abc)).unwrap();

    let mut dw1000_timer = Timer::tim9(dp.TIM9, 1.mhz(), clocks);

    let bitrate = BitRate::Kbps110;
    let preamble = PreambleLength::Symbols1536;
    let sfd_sequence = SfdSequence::Decawave;
    let pulse_repetition_frequency = PulseRepetitionFrequency::Mhz16;

    const DATA_LEN: usize = 1012;
    let mut data = [0u8; DATA_LEN];
    let mut counter = 0u8;
    for i in 0..DATA_LEN {
        data[i] = counter;
        if counter == 255 {
            counter = 0;
        } else {
            counter += 1;
        }
    }

    let mut buffer = [0; 1536];

    let mut alive_indicator = 0i8;
    let mut saw_max = 1;
    use core::num::Wrapping;
    let mut seq = Wrapping(0u8);
    loop {
        #[cfg(feature = "master")] {
            let tx_config = TxConfig {
                bitrate,
                preamble_length: preamble,
                sfd_sequence,
                pulse_repetition_frequency,
                ..TxConfig::default()
            };
            dw1000.enable_tx_interrupts();

            trace_pin.set_high().ok();

            let current_data_len = match saw_max {
                1 => {1},
                2 => {16},
                3 => {32},
                4 => {64},
                5 => {128},
                6 => {256},
                7 => {384},
                8 => {512},
                9 => {1010},
                _ => {1}
            };
            use dw1000::mac;
            use dw1000::mac::{Frame, Header};
            let frame = mac::Frame {
                header: mac::Header {
                    frame_type:      mac::FrameType::Data,
                    version:         mac::FrameVersion::Ieee802154_2006,
                    security:        mac::Security::None,
                    frame_pending:   false,
                    ack_request:     false,
                    pan_id_compress: false,
                    destination:     mac::Address::broadcast(&mac::AddressMode::Short),
                    source:          mac::Address::Short(mac::PanId(0x4321), mac::ShortAddress(0x9abc)),
                    seq:             seq.0,
                },
                content: mac::FrameContent::Data,
                payload: &data[..current_data_len],
                footer: [0; 2],
            };
            seq += Wrapping(1u8);
            let len = frame.encode(&mut buffer, mac::WriteFooter::No);
            let mut sending = dw1000.send_raw(&buffer[0..len], None, tx_config).unwrap();
            //let mut sending = dw1000.send(&data[..saw_max], mac::Address::broadcast(&mac::AddressMode::Short), None, tx_config).unwrap();

            trace_pin.set_low().ok();
            cortex_m::asm::nop();
            trace_pin.set_high().ok();
            let result = block!(sending.wait());
            trace_pin.set_low().ok();
            cortex_m::asm::nop();
            trace_pin.set_high().ok();
            dw1000 = sending.finish_sending().unwrap();
            trace_pin.set_low().ok();

            rprintln!("tx:{}: {:?}", saw_max, result);

            led1_green.set_high().ok();
            delay.delay_ms(10_u32);
            led1_green.set_low().ok();
            delay.delay_ms(10_u32);

            if saw_max == 9 {
                saw_max = 1;
                delay.delay_ms(1000_u32);
            } else {
                saw_max += 1;
            }
        }
        #[cfg(feature = "slave")] {
            let rx_config = RxConfig {
                bitrate,
                expected_preamble_length: preamble,
                sfd_sequence,
                frame_filtering: false,
                pulse_repetition_frequency,
                ..RxConfig::default()
            };
            trace_pin.set_high().ok();
            let mut receiving = dw1000.receive(rx_config).unwrap();
            trace_pin.set_low().ok();
            cortex_m::asm::nop();
            trace_pin.set_high().ok();

            dw1000_timer.start(10.hz()); //1ms?
            let result = block_timeout!(&mut dw1000_timer, receiving.wait(&mut buffer));
            trace_pin.set_low().ok();
            cortex_m::asm::nop();
            trace_pin.set_high().ok();

            dw1000 = receiving.finish_receiving().unwrap();
            trace_pin.set_low().ok();

            match result {
                Ok(m) => {
                    let frame = m.0.frame;
                    //rprintln!("dest:{:?}\tsource:{:?}\tseq:{}\tdata:{:?}", frame.header.destination, frame.header.source, frame.header.seq, frame.payload);
                    // rprint!("{:04} d:["/*, frame.header.seq, */,frame.payload.len());
                    // for b in frame.payload {
                    //     rprint!("{:02x}", b);
                    // }
                    // rprintln!("]");
                    rprintln!("{:04} {}", frame.payload.len(), check_saw(frame.payload));
                    led1_green.set_high().unwrap();
                },
                Err(e) => {
                    rprintln!("rxE: {:?}", e);
                    led1_red.set_high().unwrap();
                }
            }
            delay.delay_ms(5_u32);
            led1_green.set_low().unwrap();
            led1_red.set_low().unwrap();
        }

        // if alive_indicator >= 0 {
        //     rprint!(".");
        //     alive_indicator = alive_indicator + 1;
        //     if alive_indicator == 30 {
        //         rprintln!();
        //         alive_indicator = 0;
        //     }
        // }
    }
}

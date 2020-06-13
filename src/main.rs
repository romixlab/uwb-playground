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

use rtic::app;
use rtic::cyccnt::U32Ext;
use stm32f4xx_hal as hal;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::gpio::{PushPull, Output, Alternate, AF5};
use stm32f4xx_hal::gpio::{gpiob::{PB4, PB5, PB8, PB9}};
use rtt_target::{rtt_init_print, rprintln, rprint};
use cortex_m::asm::delay;
use hal::stm32::Interrupt;
use cortex_m::peripheral::DWT;
use heapless::{
    consts::U4,
    i,
    spsc::{Consumer, Producer, Queue}
};
use hal::{
    spi::Spi,
};
use embedded_hal::spi::MODE_0;
use dw1000::{DW1000, mac, configs::{
                RxConfig,
                TxConfig,
                SfdSequence,
                BitRate,
                PreambleLength},
             Ready, Receiving, Sending};
use hal::gpio::{gpioa::{PA4, PA5, PA6, PA7}};

pub enum DW1000State<SPI, CS> {
    Ready(DW1000<SPI, CS, Ready>),
    Sending(DW1000<SPI, CS, Sending>),
    Receiving(DW1000<SPI, CS, Receiving>),
}

type DW1000_CLK = PA5<Alternate<AF5>>;
type DW1000_MISO = PA6<Alternate<AF5>>;
type DW1000_MOSI = PA7<Alternate<AF5>>;
type DW1000_CS = PA4<Output<PushPull>>;
type DW1000_SPI = hal::spi::Spi<hal::stm32::SPI1, (DW1000_CLK, DW1000_MISO, DW1000_MOSI)>;
type UWBRadio = DW1000State<DW1000_SPI, DW1000_CS>;

const PERIOD: u32 = 10_000_000;

#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        uwb: UWBRadio,
        led1_red: PB4<Output<PushPull>>,
        led1_green: PB5<Output<PushPull>>,
        led2_red: PB8<Output<PushPull>>,
        led2_green: PB9<Output<PushPull>>,
        p: Producer<'static, u8, U4>,
        c: Consumer<'static, u8, U4>
    }

    #[init(schedule = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut Q: Queue<u8, U4> = Queue(i::Queue::new());

        rtt_init_print!(NoBlockSkip);
        rprintln!("\x1b[2J\x1b[0m");
        rprintln!("init()");

        let mut core/*: cortex_m::Peripherals */= cx.core;
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let device: stm32f4xx_hal::stm32::Peripherals = cx.device;
        // device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
        //let _flash = device.FLASH;
        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        //let mut delay = hal::delay::Delay::new(core.SYST, clocks);

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();

        let mut led1_red = gpiob.pb4.into_push_pull_output();
        let mut led1_green = gpiob.pb5.into_push_pull_output();
        let mut led2_red = gpiob.pb8.into_push_pull_output();
        let mut led2_green = gpiob.pb9.into_push_pull_output();
        led1_green.set_low().ok();
        led1_red.set_low().ok();
        led2_green.set_high().ok();
        led2_red.set_low().ok();

        // DW1000
        let mut dw1000_reset  = gpiob.pb0.into_open_drain_output(); // open drain, do not pull high
        let mut dw1000_cs = gpioa.pa4.into_push_pull_output();
        dw1000_cs.set_high().ok();
        let dw1000_clk    = gpioa.pa5.into_alternate_af5();
        let dw1000_mosi   = gpioa.pa7.into_alternate_af5();
        let dw1000_miso   = gpioa.pa6.into_alternate_af5();
        let _dw1000_wakeup = gpioc.pc5;
        let _dw1000_irq    = gpioc.pc4;
        let dw1000_spi_freq = 3.mhz();
        let dw1000_spi = Spi::spi1(
            device.SPI1,(dw1000_clk, dw1000_miso, dw1000_mosi),
            MODE_0,
            dw1000_spi_freq.into(),
            clocks);
        let mut dw1000 = DW1000::new(dw1000_spi, dw1000_cs);
        //let _:() = dw1000;

        dw1000_reset.set_low().ok();
        delay(clocks.sysclk().0 / 1000 * 2);
        dw1000_reset.set_high().ok();
        delay(clocks.sysclk().0 / 1000 * 5);
        let mut dw1000 = dw1000.init().unwrap();
        dw1000.set_address(mac::PanId(0x0d57), mac::ShortAddress(0xaabb)).unwrap();

        // rtic::pend(Interrupt::EXTI1);
        cx.schedule.blinker(cx.start + PERIOD.cycles()).unwrap();

        let (p, c) = Q.split();
        rprintln!("init(): done");
        init::LateResources {
            uwb: UWBRadio::Ready(dw1000),
            led1_green, led1_red, led2_green, led2_red,
            p, c
        }
    }

    #[idle(resources = [led2_red, c])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            if let Some(byte) = cx.resources.c.dequeue() {
                rprintln!("dequeue: {}\n", byte);
            }
            atomic::compiler_fence(Ordering::SeqCst);
        }
    }

    #[task(resources = [led1_green, p], schedule = [blinker])]
    fn blinker(cx: blinker::Context) {
        static mut LED_STATE: bool = false;
        static mut DOT_COUNTER: u8 = 0;
        rprint!(=> 2, ".");
        *DOT_COUNTER += 1;
        if *DOT_COUNTER == 30 {
            rprintln!(=> 2, "");
            *DOT_COUNTER = 0;
        }
        cx.resources.p.enqueue(*DOT_COUNTER).unwrap();

        if *LED_STATE {
            cx.resources.led1_green.set_low().ok();
            *LED_STATE = false;
        } else {
            cx.resources.led1_green.set_high().ok();
            *LED_STATE = true;
        }
        cx.schedule.blinker(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task]
    fn radio(_cx: radio::Context) {
        //     loop {
//         #[cfg(feature = "tx")] {
//             let tx_config = TxConfig {
//                 bitrate,
//                 preamble_length: preamble,
//                 sfd_sequence,
//                 ..TxConfig::default()
//             };
//             let mut sending = dw1000.send(b"ping", mac::Address::broadcast(&mac::AddressMode::Short), None, tx_config).unwrap();
//             let result = block!(sending.wait());
//             rprintln!("tx:{:?}", result);
//             dw1000 = sending.finish_sending().unwrap();
//             led1_green.set_high().unwrap();
//             delay.delay_ms(25_u32);
//             led1_green.set_low().unwrap();
//             delay.delay_ms(25_u32);
//         }
//         #[cfg(feature = "rx")] {
//             let rx_config = RxConfig {
//                 bitrate,
//                 expected_preamble_length: preamble,
//                 sfd_sequence,
//                 ..RxConfig::default()
//             };
//             let mut receiving = dw1000.receive(rx_config).unwrap();
//             let mut buffer = [0; 64];
//             dw1000_timer.start(10.hz()); //1ms?
//             let result = block_timeout!(&mut dw1000_timer, receiving.wait(&mut buffer));
//             dw1000 = receiving.finish_receiving().unwrap();
//             match result {
//                 Ok(m) => {
//                     let frame = m.frame;
//                     rprintln!("dest:{:?}\tsource:{:?}\tseq:{}\tdata:{:?}", frame.header.destination, frame.header.source, frame.header.seq, frame.payload);
//                     led1_green.set_high().unwrap();
//                 },
//                 Err(e) => {
//                     rprintln!("rxE: {:?}", e);
//                     led1_red.set_high().unwrap();
//                 }
//             }
    }

    extern "C" {
        fn EXTI0();
    }
};

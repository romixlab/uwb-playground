use crate::board::hal;
use crate::radio;
use crate::config;
use crate::crc_framer;

use bbqueue::{BBBuffer};
use rtt_target::{rtt_init_print, rprint, rprintln};
use cortex_m::peripheral::DWT;
use hal::gpio::{Edge, ExtiPin};
use dw1000::DW1000;
use hal::spi::Spi;
use embedded_hal::digital::v2::OutputPin;

pub fn init(
    cx: crate::init::Context,
    radio_commands_queue: &'static mut radio::CommandQueue,
    vesc_bbbuffer: &'static mut BBBuffer<config::VescBBBufferSize>,
    ctrl_bbbuffer: &'static mut BBBuffer<config::CtrlBBBufferSize>,
    lidar_queue: &'static mut crate::rplidar::LidarQueue,
) -> crate::init::LateResources {
    rtt_init_print!(NoBlockSkip);
    //rprintln!("\x1b[2J\x1b[0m");
    rprint!("UWB v");
    rprintln!(env!("CARGO_PKG_VERSION"));
    rprintln!("\x1b[1;36;40minit()\x1b[0m ");

    let mut core/*: cortex_m::Peripherals */= cx.core;
    core.DCB.enable_trace();
    DWT::unlock();
    core.DWT.enable_cycle_counter();

    let device: hal::stm32::Peripherals = cx.device;
    //let _flash = device.FLASH;
    use hal::rcc::RccExt;
    let rcc = device.RCC.constrain();
    use hal::time::U32Ext;
    // If you change clock frequency, make sure to also change tracer sysclk!
    cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let clocks = rcc.cfgr.sysclk(72.mhz()).freeze();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut flash = device.FLASH.constrain();
                let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);
                let clocks = rcc.cfgr.sysclk(72.mhz()).freeze(&mut flash.acr, &mut pwr);
            }
        }
    let mut syscfg = device.SYSCFG;
    let mut exti = device.EXTI;

    use hal::gpio::GpioExt;
    cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let gpioa = device.GPIOA.split();
                let gpiob = device.GPIOB.split();
                let gpioc = device.GPIOC.split();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);
                let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
                let mut gpioc = device.GPIOC.split(&mut rcc.ahb2);
            }
        }

    cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let mut led1_red = gpiob.pb4.into_push_pull_output();
                let mut led_blinky = gpiob.pb5.into_push_pull_output();
                let mut led2_red = gpiob.pb8.into_push_pull_output();
                let mut led2_green = gpiob.pb9.into_push_pull_output();
                led1_red.set_low().ok();
                led2_green.set_high().ok();
                led2_red.set_low().ok();
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut led_blinky = gpioc.pc10.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
            }
        }
    led_blinky.set_low().ok();

    // DW1000
    let dw1000_spi_freq = 2.mhz();
    let dw1000_spi_freq_hi = 20.mhz();

    cfg_if::cfg_if! {
            if #[cfg(feature = "pozyx-board")] {
                let mut dw1000_reset  = gpiob.pb0.into_open_drain_output(); // open drain, do not pull high
                let mut dw1000_cs = gpioa.pa4.into_push_pull_output();
                let dw1000_clk    = gpioa.pa5.into_alternate_af5();
                let dw1000_mosi   = gpioa.pa7.into_alternate_af5();
                let dw1000_miso   = gpioa.pa6.into_alternate_af5();
                let _dw1000_wakeup = gpioc.pc5;
                let mut dw1000_irq = gpioa.pa0.into_pull_down_input(); // Header pin 2 jump wired to IRQ pin
                //let mut dw1000_irq    = gpioc.pc4.into_pull_down_input(); // IRQ never ends with this
                let trace_pin = gpioa.pa1.into_push_pull_output(); // Header pin 1
                dw1000_irq.make_interrupt_source(&mut syscfg);
                dw1000_irq.trigger_on_edge(&mut exti, Edge::RISING);
                dw1000_irq.enable_interrupt(&mut exti);

                let dw1000_spi = Spi::spi1(
                    device.SPI1,(dw1000_clk, dw1000_miso, dw1000_mosi),
                    embedded_hal::spi::MODE_0,
                    dw1000_spi_freq.into(),
                    clocks
                );
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut dw1000_reset  = gpioc.pc11.into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper); // open drain, do not pull high
                let mut dw1000_cs = gpioa.pa8.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
                let dw1000_clk    = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                let dw1000_mosi   = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                let dw1000_miso   = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
                //let _dw1000_wakeup = gpioc.pc5;
                let trace_pin = gpioc.pc8.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
                let mut dw1000_irq = gpioc.pc9.into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);
                let mut dw1000_spi = Spi::spi1(
                    device.SPI1,
                    (dw1000_clk, dw1000_miso, dw1000_mosi),
                    MODE_0,
                    dw1000_spi_freq,
                    clocks,
                    &mut rcc.apb2,
                );
            }
        }
    dw1000_cs.set_high().ok();
    dw1000_reset.set_low().ok();
    busywait!(ms_alt, clocks, 2);
    dw1000_reset.set_high().ok();
    busywait!(ms_alt, clocks, 5);
    let dw1000 = DW1000::new(dw1000_spi, dw1000_cs);
    let dw1000 = dw1000.init(dw1000::configs::MaximumFrameLength::Decawave1023);
    let mut dw1000 = match dw1000 {
        Ok(dw1000) => { dw1000 },
        Err(e) => { panic!("DW1000 init error {:?}", e); }
    };
    for _ in 0..10 {
        let sys_status = dw1000.ll().sys_status().read().unwrap();
        if sys_status.clkpll_ll() == 0b0 {
            rprint!("SPI speed bump to: {}MHz, ", dw1000_spi_freq_hi.0);
            dw1000.ll().access_spi(|spi| {
                let (old_spi, pins) = spi.free();
                Spi::spi1(
                    old_spi, pins,
                    embedded_hal::spi::MODE_0,
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
            busywait!(ms_alt, clocks, 10);
        }
    }
    dw1000.set_address(config::PAN_ID, config::UWB_ADDR).unwrap();
    dw1000.configure_leds(false, false, true, true, 1).unwrap();

    // To VESC
    use hal::serial::{Serial, Event, config::Config};
    let usart1_tx = gpiob.pb6.into_alternate_af7();
    let usart1_rx = gpiob.pb7.into_alternate_af7();
    let mut vesc_serial = Serial::usart1(
        device.USART1,
        (usart1_tx, usart1_rx),
        Config::default().baudrate(115_200.bps()),
        clocks
    ).unwrap();
    vesc_serial.listen(Event::Rxne);
    let vesc_framer = crc_framer::CrcFramerDe::new();
    let (vesc_bbbuffer_p, vesc_bbbuffer_c) = vesc_bbbuffer.try_split().unwrap();

    // To Ctrl or lidar
    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                let ctrl_baudrate = 115_200.bps();
            } else {
                let ctrl_baudrate = 256_000.bps();
            }
        }
    let usart2_tx = gpioa.pa2.into_alternate_af7();
    let usart2_rx = gpioa.pa3.into_alternate_af7();
    let mut ctrl_serial = Serial::usart2(
        device.USART2,
        (usart2_tx, usart2_rx),
        Config::default().baudrate(ctrl_baudrate),
        clocks
    ).unwrap();
    ctrl_serial.listen(Event::Rxne);
    let ctrl_framer = crc_framer::CrcFramerDe::new();
    let (ctrl_bbbuffer_p, ctrl_bbbuffer_c) = ctrl_bbbuffer.try_split().unwrap();

    let usart6_tx = gpioc.pc6.into_alternate_af8();
    let lift_serial = Serial::usart6(
        device.USART6,
        (usart6_tx, hal::serial::NoRx),
        Config::default().baudrate(115_200.bps()),
        clocks
    ).unwrap();

    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            let mecanum_wheels = crate::motion::MecanumWheels::default();
        } else if  #[cfg(feature = "slave")] {
            let wheel = crate::motion::MCData::default();
        }
    }

    rprintln!("init(): done");

    cx.spawn.blinker().expect("RTIC failure?");
    cx.spawn.radio_chrono().expect("RTIC failure?");
    cfg_if::cfg_if! {
        if #[cfg(feature = "br")] { // lidar ctrl
            cx.spawn.ctrl_link_control().unwrap();
        }
    }
    let (radio_commands_p, radio_commands_c) = radio_commands_queue.split();
    let (lidar_queue_p, lidar_queue_c) = lidar_queue.split();
    let radio_queues = crate::tasks::radio::DataQueues::new();

    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                crate::init::LateResources {
                    clocks,

                    radio_state: radio::RadioState::Ready(Some(dw1000)),
                    radio_queues,
                    radio_irq: dw1000_irq,
                    radio_trace: trace_pin,
                    radio_commands_p, radio_commands_c,

                    vesc_serial,
                    vesc_framer,
                    vesc_bbbuffer_p, vesc_bbbuffer_c,

                    ctrl_serial,
                    ctrl_framer,
                    ctrl_bbbuffer_p, ctrl_bbbuffer_c,

                    lift_serial,

                    mecanum_wheels,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,

                    stat: radio::Stat::default()
                }
            } else if #[cfg(any(feature = "tr", feature = "bl"))] {
                crate::init::LateResources {
                    clocks,

                    radio_state: radio::RadioState::Ready(Some(dw1000)),
                    radio_irq: dw1000_irq,
                    radio_trace: trace_pin,
                    radio_commands_p, radio_commands_c,

                    vesc_serial,
                    vesc_framer,
                    vesc_bbbuffer_p, vesc_bbbuffer_c,

                    ctrl_serial,
                    ctrl_framer,
                    ctrl_bbbuffer_p, ctrl_bbbuffer_c,

                    lift_serial,

                    wheel,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti
                }
            } else if #[cfg(feature = "br")] {
                crate::init::LateResources {
                    clocks,

                    radio_state: radio::RadioState::Ready(Some(dw1000)),
                    radio_irq: dw1000_irq,
                    radio_trace: trace_pin,
                    radio_commands_p, radio_commands_c,

                    vesc_serial,
                    vesc_framer,
                    vesc_bbbuffer_p, vesc_bbbuffer_c,

                    ctrl_serial,
                    ctrl_framer,
                    ctrl_bbbuffer_p, ctrl_bbbuffer_c,

                    lift_serial,

                    wheel,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,

                    lidar: crate::rplidar::RpLidar::new(),
                    lidar_queue_p,
                    lidar_queue_c
                }
            }
        }
}

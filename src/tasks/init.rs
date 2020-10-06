use crate::board::hal;
use crate::radio;
use crate::config;
use crate::color;

use bbqueue::{BBBuffer};
use rtt_target::{rtt_init_print, rprint, rprintln};
use cortex_m::peripheral::DWT;
#[cfg(feature = "pozyx-board")]
use hal::gpio::{Edge, ExtiPin};
use dw1000::DW1000;
use embedded_hal::digital::v2::OutputPin;
use cfg_if::cfg_if;
use radio::types::Event;
use radio::scheduler::Scheduler;
use radio::types::RadioConfig;
use crate::hal::spi::Spi;

use core::sync::atomic::{
    compiler_fence,
    Ordering
};
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::spi::FullDuplex;
use stm32g4xx_hal::gpio::SignalEdge;
use crate::radio::types::{ReadyRadio, Pong};
use dw1000::mac::{Address, PanId, ShortAddress};
use dw1000::hl::SendTime;

pub fn init(
    cx: crate::init::Context,
    radio_commands_queue: &'static mut radio::types::CommandQueue,
) -> crate::init::LateResources {
    // rtt_init_print!(NoBlockSkip);
    let rtt_channels = rtt_target::rtt_init! {
        up: {
            0: { // channel number
                size: 1024 // buffer size in bytes
                mode: NoBlockSkip // mode (optional, default: NoBlockSkip, see enum ChannelMode)
                name: "Terminal" // name (optional, default: no name)
            }
        }
        down: {
            0: {
                size: 128
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(rtt_channels.up.0);
    //rprintln!("\x1b[2J\x1b[0m");
    rprint!("\n{}==============\n= UWB\n", color::CYAN);
    rprintln!("= {}", config::DEVICE_NAME);
    rprintln!("= v{}", env!("CARGO_PKG_VERSION"));
    rprintln!("= {:?}", config::DEFAULT_UWB_CHANNEL);
    rprintln!("=============={}\n", color::DEFAULT);
    rprintln!("init...");

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
    cfg_if! {
        if #[cfg(feature = "pozyx-board")] {
            let clocks = rcc.cfgr.sysclk(72.mhz()).freeze();
        } else if #[cfg(feature = "gcharger-board")] {
            let mut rcc = rcc.freeze(stm32g4xx_hal::rcc::Config::default());
            let clocks = rcc.clocks;
        }
    }
    rprintln!("Clocks: {:#?}", clocks);
    let mut syscfg = device.SYSCFG;
    let mut exti = device.EXTI;

    use hal::gpio::GpioExt;
    cfg_if! {
        if #[cfg(feature = "pozyx-board")] {
            let gpioa = device.GPIOA.split();
            let gpiob = device.GPIOB.split();
            let gpioc = device.GPIOC.split();
        } else if #[cfg(feature = "gcharger-board")] {
            let mut gpioa = device.GPIOA.split(&mut rcc);
            let mut gpiob = device.GPIOB.split(&mut rcc);
            let mut gpioc = device.GPIOC.split(&mut rcc);
            let mut gpiod = device.GPIOD.split(&mut rcc);
        }
    }

    cfg_if! {
        if #[cfg(feature = "pozyx-board")] {
            let mut led1_red = gpiob.pb4.into_push_pull_output();
            let mut led_blinky = gpiob.pb5.into_push_pull_output();
            let mut led2_red = gpiob.pb8.into_push_pull_output();
            let mut led2_green = gpiob.pb9.into_push_pull_output();
            led1_red.set_low().ok();
            led2_green.set_high().ok();
            led2_red.set_low().ok();
        } else if #[cfg(feature = "gcharger-board")] {
            let mut led_blinky = gpiob.pb15.into_push_pull_output();
        }
    }
    led_blinky.set_low().ok();

    let fdcan1_tx = gpiob.pb9;
    let fdcan1_rx = gpiob.pb8;
    let mut can = hal::can::Can::new_classical(
        device.FDCAN1,
        (fdcan1_tx, fdcan1_rx),
        hal::can::Mode::Normal,
        hal::can::Retransmission::Enabled,
        hal::can::TransmitPause::Enabled,
        &mut rcc,
        &mut core.DWT
    );
    rprintln!("can::new_classical: {}", can.is_ok());


    // DW1000
    let dw1000_spi_freq = 1.mhz();
    let dw1000_spi_freq_hi = 18.mhz();

    cfg_if! {
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
        } else if #[cfg(feature = "gcharger-board")] {
            let mut dw1000_reset  = gpioc.pc14.into_open_drain_output(); // open drain, do not pull high
            let mut dw1000_cs = gpiod.pd2.into_push_pull_output();
            let dw1000_clk    = gpioc.pc10; // SPI3
            let dw1000_mosi   = gpioc.pc12;
            let dw1000_miso   = gpioc.pc11;
            //let _dw1000_wakeup = gpioc.pc5;
            let trace_pin = gpioa.pa10.into_push_pull_output();
            let mut dw1000_irq = gpioc.pc15.into_pull_down_input();
            dw1000_irq = dw1000_irq.listen(SignalEdge::Rising, &mut syscfg, &mut exti);
            let mut dw1000_spi = hal::spi::Spi::spi3(
                device.SPI3,
                (dw1000_clk, dw1000_miso, dw1000_mosi),
                embedded_hal::spi::MODE_0,
                dw1000_spi_freq,
                &mut rcc,
            );
        }
    }

    dw1000_cs.set_high().ok();
    dw1000_reset.set_low().ok();
    busywait!(ms_alt, clocks, 2);
    dw1000_reset.set_high().ok();
    busywait!(ms_alt, clocks, 5);
    let mut dw1000 = DW1000::new(dw1000_spi, dw1000_cs);
    match dw1000.device_info() {
        Ok(info) => {
            if info.tag_is_correct {
                rprintln!("DW1000: ok, model:{} ver:{} rev:{}", info.model, info.version, info.revision);
            } else {
                rprintln!("{}DW1000 SPI communication error: IncorrectTag{}", color::RED, color::DEFAULT);
                panic!("DW1000 SPI broken");
            }
        },
        Err(e) => {
            rprintln!("{}DW1000 SPI communication error: {:?}{}", color::RED, e, color::DEFAULT);
            panic!("DW1000 SPI broken");
        }
    }
    let dw1000 = dw1000.init(dw1000::configs::MaximumFrameLength::Decawave1023);
    let mut dw1000 = match dw1000 {
        Ok(dw1000) => { dw1000 },
        Err(e) => { panic!("DW1000 init error {:?}", e); }
    };
    for _ in 0..10 {
        let sys_status = dw1000.ll().sys_status().read().unwrap();
        if sys_status.clkpll_ll() == 0b0 {
            rprint!("SPI speed bump to: {}MHz, ", dw1000_spi_freq_hi.0);
            cfg_if! {
                if #[cfg(feature = "pozyx-board")] {
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
                } else if #[cfg(feature = "gcharger-board")] {
                    dw1000.ll().access_spi(|spi| {
                        let (old_spi, pins) = spi.release();
                        Spi::spi3(
                            old_spi, pins,
                            embedded_hal::spi::MODE_0,
                            dw1000_spi_freq_hi,
                            &mut rcc
                        )
                    });
                    let br = unsafe {
                        let spi3 = unsafe { &(*hal::stm32::SPI3::ptr()) };
                        spi3.cr1.read().br().bits()
                    };
                }
            }
            let ratio = 1 << (br + 1);
            cfg_if! {
                if #[cfg(feature = "pozyx-board")] {
                    rprintln!("done, actual ratio:{} baud:{}", ratio, clocks.pclk2().0 / ratio);
                } else if #[cfg(feature = "gcharger-board")] {
                    rprintln!("done, actual ratio:{} baud:{}", ratio, clocks.apb1_clk.0 / ratio);
                }
            }
            break;
        } else {
            rprintln!("clkpll_ll = 1, resetting");
            busywait!(ms_alt, clocks, 10);
        }
    }
    dw1000.set_address(config::PAN_ID, config::UWB_ADDR).unwrap();
    dw1000.configure_leds(false, false, true, true, 1).unwrap();
    //dw1000.set_antenna_delay(16456, 16300).expect("Failed to set antenna delay");
    //dw1000.set_antenna_delay(16147, 16166).expect("Failed to set antenna delay");
    dw1000.set_antenna_delay(16128, 16145).expect("Failed to set antenna delay");

    let (radio_commands_p, radio_commands_c) = radio_commands_queue.split();
    let event_state_data = crate::tasks::radio::EventStateData::default();

    cx.spawn.blinker().expect("RTIC failure?");
    cfg_if! {
        if #[cfg(feature = "master")] {
            cx.spawn.radio_event(
                radio::Event::GTSAboutToStart(
                    Scheduler::gts_phase_duration(), RadioConfig::default()
                )).ok();
        } else if #[cfg(any(feature = "slave", feature = "anchor"))] {
            cx.spawn.radio_event(radio::Event::GTSStartAboutToBeBroadcasted).ok();
            cx.spawn.radio_event(radio::Event::ReceiveCheck).ok();
        }
    }

    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

                    usart1_coder,
                    usart1_dma_tcx,
                    usart1_dma_rcx,
                    usart1_decoder,

                    usart2_coder,
                    usart2_dma_tcx,
                    usart2_dma_rcx,
                    usart2_decoder,

                    lift_serial,

                    mecanum_wheels,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,

                    lidar_frame_c,
                    motion_channel_p,
                    motion_telemetry_c,
                    local_motion_telemetry_p,

                    rtt_down_channel: rtt_channels.down.0,
                }
            } else if #[cfg(any(feature = "tr", feature = "bl"))] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

                    usart1_coder,
                    usart1_dma_tcx,
                    usart1_dma_rcx,
                    usart1_decoder,

                    lift_serial,

                    wheel,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,

                    motion_channel_c,
                    motion_telemetry_p,

                    rtt_down_channel: rtt_channels.down.0,
                }
            } else if #[cfg(feature = "br")] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

                    usart1_coder,
                    usart1_dma_tcx,
                    usart1_dma_rcx,
                    usart1_decoder,

                    usart2_c,
                    usart2_p,
                    usart2_dma_tcx,
                    usart2_dma_rcx,

                    lift_serial,

                    wheel,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,

                    lidar: crate::rplidar::RpLidar::new(lidar_frame_p),
                    motion_channel_c,
                    motion_telemetry_p,

                    rtt_down_channel: rtt_channels.down.0,
                }
            } else if #[cfg(feature = "anchor")] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    event_state_data,
                    channels: crate::channels::Channels{},

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),

                    rtt_down_channel: rtt_channels.down.0,
                }
            }
        }
}

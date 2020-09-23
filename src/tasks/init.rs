use crate::board::hal;
use crate::radio;
use crate::config;
use crate::codec;
use crate::color;

use bbqueue::{BBBuffer};
use rtt_target::{rtt_init_print, rprint, rprintln};
use cortex_m::peripheral::DWT;
use hal::gpio::{Edge, ExtiPin};
use dw1000::DW1000;
use hal::spi::Spi;
use embedded_hal::digital::v2::OutputPin;
use cfg_if::cfg_if;
use radio::types::Event;
use radio::scheduler::Scheduler;
use radio::types::RadioConfig;

use core::sync::atomic::{
    compiler_fence,
    Ordering
};

pub fn init(
    cx: crate::init::Context,
    radio_commands_queue: &'static mut radio::types::CommandQueue,
    lidar_bbuffer: &'static mut crate::rplidar::LidarBBuffer,
    usart1_dma_rx_buffer: &'static mut config::Usart1DmaRxBuffer,
    usart1_dma_tx_buffer: &'static mut config::Usart1DmaTxBuffer,
    usart2_dma_rx_buffer: &'static mut config::Usart2DmaRxBuffer,
    usart2_dma_tx_buffer: &'static mut config::Usart2DmaTxBuffer,
    motion_channel: &'static mut crate::motion::Channel,
    telemetry_channel: &'static mut crate::motion::TelemetryChannel,
    local_telemetry_channel: &'static mut crate::motion::TelemetryLocalChannel, // only for master
) -> crate::init::LateResources {
    rtt_init_print!(NoBlockSkip);
    //rprintln!("\x1b[2J\x1b[0m");
    rprint!("\n{}==============\n= UWB\n", color::CYAN);
    rprintln!("= {}", config::DEVICE_NAME);
    rprintln!("= v{}", env!("CARGO_PKG_VERSION"));
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
            } else if #[cfg(feature = "dragonfly-board")] {
                let mut flash = device.FLASH.constrain();
                let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);
                let clocks = rcc.cfgr.sysclk(72.mhz()).freeze(&mut flash.acr, &mut pwr);
            }
        }
    let mut syscfg = device.SYSCFG;
    let mut exti = device.EXTI;

    use hal::gpio::GpioExt;
    cfg_if! {
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

    cfg_if! {
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
    let mut usart1 = Serial::usart1(
        device.USART1,
        (usart1_tx, usart1_rx),
        Config::default().baudrate(config::USART1_BAUD.bps()),
        clocks
    ).unwrap();
    let (mut usart1_dma_rx_buffer_p, usart1_c) = usart1_dma_rx_buffer.try_split().unwrap();
    let (mut usart1_dma_rcx, usart1_dma_mem_addr) =
        crate::tasks::dma::DmaRxContext::new(
            usart1_dma_rx_buffer_p,
            hal::stm32::USART1::ptr(),
            hal::stm32::DMA2::ptr(),
            5
        );
    let (mut usart1_p, usart1_dma_tx_buffer_c) = usart1_dma_tx_buffer.try_split().unwrap();
    let usart1_dma_tcx = crate::tasks::dma::DmaTxContext::new(usart1_dma_tx_buffer_c);
    let usart1_coder = codec::Usart1Coder::new(usart1_p, config::USART1_TX_DMA);
    let usart1_decoder = codec::Usart1Decoder::new(usart1_c);

    // To Ctrl or lidar
    let usart2_tx = gpioa.pa2.into_alternate_af7();
    let usart2_rx = gpioa.pa3.into_alternate_af7();
    let mut usart2 = Serial::usart2(
        device.USART2,
        (usart2_tx, usart2_rx),
        Config::default().baudrate(config::USART2_BAUD.bps()),
        clocks
    ).unwrap();
    cfg_if! {
        if #[cfg(any(feature = "master", feature = "br"))] {
            let (mut usart2_dma_rx_buffer_p, usart2_c) = usart2_dma_rx_buffer.try_split().unwrap();
            let (mut usart2_dma_rcx, usart2_dma_mem_addr) =
                crate::tasks::dma::DmaRxContext::new(
                    usart2_dma_rx_buffer_p,
                    hal::stm32::USART2::ptr(),
                    hal::stm32::DMA1::ptr(),
                    5
                );

            unsafe {
                let rcc = &(*hal::stm32::RCC::ptr());
                rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
                let usart2 = &(*hal::stm32::USART2::ptr());
                let dma1 = &(*hal::stm32::DMA1::ptr());
                let stream5 = &dma1.st[5];
                // Disable stream & Deinit
                stream5.cr.modify(|_, w| w.en().disabled());
                compiler_fence(Ordering::Acquire);

                stream5.cr.write(|w| w.bits(0));
                stream5.ndtr.write(|w| w.bits(0));
                stream5.par.write(|w| w.bits(0));
                stream5.m0ar.write(|w| w.bits(0));
                stream5.m1ar.write(|w| w.bits(0));
                // Clear interrupt pending flags
                dma1.hifcr.write(|w| w
                    .cfeif5().clear()
                    .cdmeif5().clear()
                    .cteif5().clear()
                    .chtif5().clear()
                    .ctcif5().clear()
                );
                // Clear transfer complete flag & enable DMA receive request
                usart2.sr.write(|w| w.tc().set_bit());
                usart2.cr3.modify(|_, w| w.dmar().set_bit());
                // Configure addresses and transfer size
                stream5.ndtr.write(|w| w.bits(usart2_dma_rcx.buf_size() as u32)); // number of data items (bytes in this case)
                let dr_addr: u32 = usart2 as *const _ as u32 + 0x04;
                stream5.par.write(|w| w.bits(dr_addr));  // peripheral address
                stream5.m0ar.write(|w| w.bits(usart2_dma_mem_addr)); // memory address
                stream5.m1ar.write(|w| w.bits(usart2_dma_mem_addr)); //? memory address
                // Configure DMA stream
                stream5.cr.modify(|_, w| w
                    .dmeie().disabled()   // direct mode error irq
                    .teie().disabled()    // transfer error irq
                    .htie().enabled()     // half-transfer complete irq
                    .tcie().enabled()     // transfer complete irq
                    .pfctrl().dma()       // flow controller dma/peripheral (see ref.manual)
                    .dir().peripheral_to_memory()
                    .circ().enabled()     // circular mode
                    .pinc().fixed()       // peripheral addr increment
                    .psize().bits8()      // peripheral data size
                    .pincos().psize()     // peripheral increment offset size as_psize/fixed4
                    .pburst().single()    // peripheral burst size
                    .minc().incremented() // memory addr increment
                    .msize().bits8()      // memory data size
                    .mburst().single()    // memory burst size
                    .pl().high()          // priority
                    .dbm().disabled()     // double buffer switching if en see also .ct()
                    .chsel().bits(4)      // DMA request channel for USART2 RX
                );
                compiler_fence(Ordering::Release);
                stream5.cr.modify(|_, w| w.en().enabled());
            }
        }
    }
    cfg_if! {
        if #[cfg(any(feature = "master", feature = "br"))] {
            let (mut usart2_p, usart2_dma_tx_buffer_c) = usart2_dma_tx_buffer.try_split().unwrap();
            let usart2_dma_tcx = crate::tasks::dma::DmaTxContext::new(usart2_dma_tx_buffer_c);

            unsafe {
                let rcc = &(*hal::stm32::RCC::ptr());
                rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
                let usart2 = &(*hal::stm32::USART2::ptr());

                let dma1 = &(*hal::stm32::DMA1::ptr());
                let stream6 = &dma1.st[6];
                // Disable stream & Deinit
                stream6.cr.modify(|_, w| w.en().disabled());
                stream6.cr.write(|w| w.bits(0));
                stream6.ndtr.write(|w| w.bits(0));
                stream6.par.write(|w| w.bits(0));
                stream6.m0ar.write(|w| w.bits(0));
                stream6.m1ar.write(|w| w.bits(0));
                // Clear interrupt pending flags
                dma1.hifcr.write(|w| w
                    .cfeif6().clear()
                    .cdmeif6().clear()
                    .cteif6().clear()
                    .chtif6().clear()
                    .ctcif6().clear()
                );
                // Clear transfer complete flag & enable DMA for transmission
                usart2.cr3.modify(|_, w| w.dmat().set_bit());
                // Configure addresses and transfer size
                stream6.ndtr.write(|w| w.bits(0)); // number of data items (bytes in this case)
                let dr_addr: u32 = usart2 as *const _ as u32 + 0x04;
                stream6.par.write(|w| w.bits(dr_addr));  // peripheral address
                stream6.m0ar.write(|w| w.bits(0)); // memory address
                stream6.m1ar.write(|w| w.bits(0)); //? memory address
                // Configure DMA stream
                stream6.cr.modify(|_, w| w
                    .dmeie().disabled()   // direct mode error irq
                    .teie().disabled()    // transfer error irq
                    .htie().disabled()    // half-transfer complete irq
                    .tcie().enabled()     // transfer complete irq
                    .pfctrl().dma()       // flow controller dma/peripheral (see ref.manual)
                    .dir().memory_to_peripheral()
                    .circ().disabled()    // circular mode
                    .pinc().fixed()       // peripheral addr increment
                    .psize().bits8()      // peripheral data size
                    .pincos().psize()     // peripheral increment offset size as_psize/fixed4
                    .pburst().single()    // peripheral burst size
                    .minc().incremented() // memory addr increment
                    .msize().bits8()      // memory data size
                    .mburst().single()    // memory burst size
                    .pl().high()          // priority
                    .dbm().disabled()     // double buffer switching if en see also .ct()
                    .chsel().bits(4)      // DMA request channel for USART2 TX
                );
            }
            usart2.listen(Event::Idle);
        }
    }
    cfg_if! {
        if #[cfg(feature = "master")] {
            let usart2_coder = codec::Usart2Coder::new(usart2_p, config::USART2_TX_DMA);
            let usart2_decoder = codec::Usart2Decoder::new(usart2_c);
        } else if  #[cfg(feature = "br")] {

        }
    }

    let usart6_tx = gpioc.pc6.into_alternate_af8();
    let lift_serial = Serial::usart6(
        device.USART6,
        (usart6_tx, hal::serial::NoRx),
        Config::default().baudrate(115_200.bps()),
        clocks
    ).unwrap();

    cfg_if! {
        if #[cfg(feature = "master")] {
            let mecanum_wheels = crate::motion::MecanumWheels::default();
        } else if  #[cfg(feature = "slave")] {
            let wheel = crate::motion::MCData::default();
        }
    }

    rprintln!("init(): done");

    cx.spawn.blinker().expect("RTIC failure?");
    cfg_if! {
        if #[cfg(feature = "master")] {
            cx.spawn.radio_event(
                radio::Event::GTSAboutToStart(
                    Scheduler::gts_phase_duration(), RadioConfig::default()
                )).ok();
        } else if #[cfg(feature = "slave")] {
            cx.spawn.radio_event(radio::Event::GTSStartAboutToBeBroadcasted).ok();
            cx.spawn.radio_event(radio::Event::ReceiveCheck).ok();
        }
    }

    let (motion_channel_p, motion_channel_c) = motion_channel.split();
    let (motion_telemetry_p, motion_telemetry_c) = telemetry_channel.split();
    let (local_motion_telemetry_p, local_motion_telemetry_c) = local_telemetry_channel.split();
    //let (lidar_queue_p, lidar_queue_c) = lidar_queue.split();
    let (lidar_frame_p, lidar_frame_c) = lidar_bbuffer.try_split_framed().unwrap();

    cfg_if::cfg_if! {
        if #[cfg(feature = "br")] { // lidar ctrl
            cx.spawn.ctrl_link_control().unwrap();
            let channels = crate::channels::Channels {
                lidar_bbuffer_c: lidar_frame_c,
                motion_p: motion_channel_p,
                motion_telemetry_c,
            };
        } else if #[cfg(feature = "master")] {
            let channels = crate::channels::Channels {
                lidar_bbuffer_p: lidar_frame_p,
                motion_c: motion_channel_c,
                motion_telemetry_p,
                local_motion_telemetry_c,
                telemetry_staging_area_tacho: crate::channels::TelemetryStagingAreaTacho::default(),
                telemetry_staging_area_power: crate::channels::TelemetryStagingAreaPower::default(),
            };
        } else { // TR, BL
            let channels = crate::channels::Channels {
                //lidar_queue_p,
                motion_p: motion_channel_p,
                motion_telemetry_c,
            };
        }
    }

    let (radio_commands_p, radio_commands_c) = radio_commands_queue.split();
    let event_state_data = crate::tasks::radio::EventStateData::default();


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
                    local_motion_telemetry_p
                }
            } else if #[cfg(any(feature = "tr", feature = "bl"))] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

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

                    motion_channel_c,
                    motion_telemetry_p,
                }
            } else if #[cfg(feature = "br")] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

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

                    lidar: crate::rplidar::RpLidar::new(lidar_frame_p),
                    lidar_dma,
                    lidar_dma_c: lidar_dma_buffer_c,
                    motion_channel_c,
                    motion_telemetry_p,
                }
            } else if #[cfg(feature = "anchor")] {
                crate::init::LateResources {
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    channels,
                    event_state_data,

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),
                    exti,
                }
            }
        }
}

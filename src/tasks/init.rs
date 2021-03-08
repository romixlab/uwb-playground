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

// i2c
use hal::i2c::Config;
use stm32g4xx_hal::i2c::I2cExt;
use stm32g4xx_hal::delay::Delay;
#[cfg(feature = "tof")]
use crate::vl53l1x_multi;
use crate::dump_delay;
use stm32g4xx_hal::watchdog::{IndependentWatchdog, IWDGExt};
use embedded_hal::watchdog::WatchdogEnable;
use hal::can;

pub fn init(
    cx: crate::init::Context,
    radio_commands_queue: &'static mut radio::types::CommandQueue,
    usart2_dma_rx_buffer: &'static mut config::Usart2DmaRxBuffer,
    lidar_bbuffer: &'static mut crate::rplidar::LidarBBuffer,
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

    let mut core/*: cortex_m::Peripherals*/ = cx.core;
    core.DCB.enable_trace();
    DWT::unlock();
    core.DWT.enable_cycle_counter();

    let device: hal::stm32::Peripherals = cx.device;

    //let _flash = device.FLASH;
    use hal::rcc::RccExt;
    let mut rcc = device.RCC.constrain();

    let mut iwdg = device.IWDG.constrain();
    #[cfg(not(feature = "disable-watchdogs"))]
    iwdg.start(crate::hal::time::MicroSecond(6_000_000));

    use hal::time::U32Ext;
    // If you change clock frequency, make sure to also change tracer sysclk!
    cfg_if! {
        if #[cfg(feature = "pozyx-board")] {
            let clocks = rcc.cfgr.sysclk(72.mhz()).freeze();
        } else if #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))] {
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
        } else if #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))] {
            let mut gpioa = device.GPIOA.split(&mut rcc);
            let mut gpiob = device.GPIOB.split(&mut rcc);
            let mut gpioc = device.GPIOC.split(&mut rcc);
            let mut gpiod = device.GPIOD.split(&mut rcc);
            let mut gpiof = device.GPIOF.split(&mut rcc);
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
            let mut led_red = gpioc.pc6.into_push_pull_output();
        } else if #[cfg(feature = "gcarrier-board")] {
            let mut led_blinky = gpiob.pb1.into_push_pull_output();
            let mut led_red = gpiob.pb0.into_push_pull_output();
        }
    }
    led_blinky.set_low().ok();
    led_red.set_high().ok();
    cortex_m::asm::delay(16_000_000u32);
    led_red.set_low().ok();

    cfg_if! {
        if #[cfg(feature = "gcharger-board")] {
            let fdcan1_tx = gpiob.pb9;
            let fdcan1_rx = gpiob.pb8;
        } else if #[cfg(feature = "gcarrier-board")] {
            let fdcan1_tx = gpioa.pa12;
            let fdcan1_rx = gpioa.pa11;
        }
    }
    use can::{CanController, CanInstance, ClockSource};
    let mut can_ctrl = CanController::new(
        ClockSource::Pllq,
        &mut rcc,
        &mut core.DWT
    ).unwrap();
    let mut can0 = CanInstance::new_classical(
        &mut can_ctrl,
        device.FDCAN1,
        (fdcan1_tx, fdcan1_rx),
        can::Mode::Normal,
        can::Retransmission::Enabled,
        can::TransmitPause::Disabled,
        can::TxBufferMode::Fifo,
        can::ClockDiv::Div1,
        can::BitTiming::default_1mbps(),
    );
    rprintln!("can::new_classical: {}", can0.is_ok());
    let mut can0 = match can0 {
        Ok((can0, _)) => can0,
        Err(e) => {
            panic!("can::new_classical: err: {:?}", e.0);
        }
    };
    use can::ClassicalCan;
    unsafe {
        can0.ll(|can_regs| {
            can_regs.ie.write(|w| w.bits(0xff_ffff)); // enable all interrupts
            can_regs.ils.write(|w| w.bits(0b0000_0000)); // all interrupts to irq0 line
            can_regs.ile.write(|w| w.eint0().set_bit().eint1().clear_bit()); // enable irq0 line
            can_regs.txbtie.write(|w| w.bits(0b001)); // enable tx buffer interrupt
        });
    }
    let mut can0_receive_heap = vhrdcan::FrameHeap::new();
    let can0_ll_statistics = crate::tasks::canbus::LLStatistics::default();
    let mut can0_rx_routing_table = heapless::Vec::new();
    crate::tasks::canbus::load_rx_routing_table(&mut can0_rx_routing_table);
    let can0_rx_routing_statistics = crate::tasks::canbus::RxRoutingStatistics::default();
    let can0_local_processing_heap = vhrdcan::FrameHeap::new();
    let can0_irq_statistics = crate::tasks::canbus::IrqStatistics::default();
    let can0_analyzer = crate::tasks::canbus::CanAnalyzer::new();

    cfg_if! {
        if #[cfg(feature = "tof")] {
            let mut delay = dump_delay::DumpDelay::new();
            //i2c1
            let sda = gpiob.pb7.into_open_drain_output();
            let scl = gpioa.pa15.into_open_drain_output();
            let mut i2c: config::I2cPortType = device
                .I2C1
                .i2c(sda, scl, Config::with_timing(0x2020151b), &mut rcc);

            rprintln!("before init tof");
            let mut vl53l1_multi = vl53l1x_multi::Vl53l1Multi::new(i2c, delay, [0,1,2,3], 0x70);
            rprintln!("tof::new {}");
            vl53l1_multi.init_devices();
            rprintln!("tof::init_devs {}");
        }
    }

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
            let mut dw1000_irq = gpiob.pb11.into_pull_down_input(); // Header pin 2 jump wired to IRQ pin
            //let mut dw1000_irq    = gpioc.pc4.into_pull_down_input(); // IRQ never ends with this
            let trace_pin = gpioa.pa2.into_push_pull_output(); // unsafe bit banged in radio.rs
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
            let trace_pin = gpioa.pa2.into_push_pull_output();
            let mut dw1000_irq = gpioc.pc15.into_pull_down_input();
            dw1000_irq = dw1000_irq.listen(SignalEdge::Rising, &mut syscfg, &mut exti, &mut rcc);
            let mut dw1000_spi = hal::spi::Spi::spi3(
                device.SPI3,
                (dw1000_clk, dw1000_miso, dw1000_mosi),
                embedded_hal::spi::MODE_0,
                dw1000_spi_freq,
                &mut rcc,
            );
        } else if #[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))] {
            let mut dw1000_reset  = gpioa.pa4.into_open_drain_output(); // open drain, do not pull high
            let mut dw1000_cs = gpioa.pa1.into_push_pull_output();
            let dw1000_clk    = gpioa.pa5; // SPI1
            let dw1000_mosi   = gpioa.pa7;
            let dw1000_miso   = gpioa.pa6;
            let mut dw1000_irq = gpioa.pa8.into_pull_down_input();

            //let _dw1000_wakeup = gpioa.pa9;
            let trace_pin = gpioc.pc3.into_push_pull_output();

            dw1000_irq = dw1000_irq.listen(SignalEdge::Rising, &mut syscfg, &mut exti, &mut rcc);
            let mut dw1000_spi = hal::spi::Spi::spi1(
                device.SPI1,
                (dw1000_clk, dw1000_miso, dw1000_mosi),
                embedded_hal::spi::MODE_0,
                dw1000_spi_freq,
                &mut rcc,
            );
        } else if #[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))] {
            let mut dw1000_reset  = gpioc.pc6.into_open_drain_output(); // open drain, do not pull high
            let mut dw1000_cs = gpioc.pc7.into_push_pull_output();
            let dw1000_clk    = gpiof.pf1; // SPI2
            let dw1000_mosi   = gpiob.pb15;
            let dw1000_miso   = gpiob.pb14;
            let mut dw1000_irq = gpioc.pc8.into_pull_down_input();
            let trace_pin = gpioc.pc3.into_push_pull_output();

            dw1000_irq = dw1000_irq.listen(SignalEdge::Rising, &mut syscfg, &mut exti, &mut rcc);
            let mut dw1000_spi = hal::spi::Spi::spi2(
                device.SPI2,
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
    use dw1000::hl::{TxPowerControl, ManualPowerGain, PowerGain, CoarsePowerGain, FinePowerGain};
    let dw1000_power_gain = PowerGain {
        coarse: CoarsePowerGain::_15dB,
        fine: FinePowerGain::_15dB
    };
    let dw1000_tx_power = TxPowerControl::Manual(ManualPowerGain {
        phy_header: dw1000_power_gain,
        shr_and_data: dw1000_power_gain
    });
    let dw1000 = dw1000.init(dw1000_tx_power, dw1000::configs::MaximumFrameLength::Decawave1023);
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
                } else if #[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))] {
                    dw1000.ll().access_spi(|spi| {
                        let (old_spi, pins) = spi.release();
                        Spi::spi1(
                            old_spi, pins,
                            embedded_hal::spi::MODE_0,
                            dw1000_spi_freq_hi,
                            &mut rcc
                        )
                    });
                    let br = unsafe {
                        let spi1 = unsafe { &(*hal::stm32::SPI1::ptr()) };
                        spi1.cr1.read().br().bits()
                    };
                } else if #[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))] {
                    dw1000.ll().access_spi(|spi| {
                        let (old_spi, pins) = spi.release();
                        Spi::spi2(
                            old_spi, pins,
                            embedded_hal::spi::MODE_0,
                            dw1000_spi_freq_hi,
                            &mut rcc
                        )
                    });
                    let br = unsafe {
                        let spi2 = unsafe { &(*hal::stm32::SPI2::ptr()) };
                        spi2.cr1.read().br().bits()
                    };
                }
            }
            let ratio = 1 << (br + 1);
            cfg_if! {
                if #[cfg(feature = "pozyx-board")] {
                    rprintln!("done, actual ratio:{} baud:{}", ratio, clocks.pclk2().0 / ratio);
                } else if #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))] {
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
    dw1000.configure_leds(true, false, true, true, 1, false, false, false).unwrap();
    //dw1000.set_antenna_delay(16456, 16300).expect("Failed to set antenna delay");
    //dw1000.set_antenna_delay(16147, 16166).expect("Failed to set antenna delay");
    dw1000.set_antenna_delay(16128, 16145).expect("Failed to set antenna delay");
    cfg_if! {
        if #[cfg(feature = "gcarrier-board")] {
            let mut dw_b_select = gpioa.pa10.into_push_pull_output(); // DW_A_SELECT in sch, RF paths are swapped, ok
            let mut ant_a_or_bc_select = gpioc.pc0.into_push_pull_output();
            let mut ant_b_or_c_select = gpioc.pc1.into_push_pull_output();

            cfg_if! {
                if #[cfg(feature = "uwb-a")] {
                    dw_b_select.set_low().ok(); // UWB-A
                } else if #[cfg(feature = "uwb-b")] {
                    dw_b_select.set_high().ok(); // UWB-B
                }
            }
            ant_a_or_bc_select.set_high().ok(); // Ant A
        }
    }

    let (radio_commands_p, radio_commands_c) = radio_commands_queue.split();
    let event_state_data = crate::tasks::radio::EventStateData::default();

    cx.spawn.blinker().expect("RTIC failure?");
    cfg_if! {
        if #[cfg(feature = "master")] {
            cx.spawn.radio_event(
                radio::Event::GTSAboutToStart(
                    Scheduler::gts_phase_duration(), RadioConfig::fast()
                )).ok();
        } else if #[cfg(any(feature = "slave", feature = "anchor"))] {
            cx.spawn.radio_event(radio::Event::GTSStartAboutToBeBroadcasted).ok();
            cx.spawn.radio_event(radio::Event::ReceiveCheck).ok();
        }
    }

    let imx_terminal_tx = gpiob.pb3;
    let imx_terminal_rx = gpioa.pa15;
    use hal::time::Bps;
    let mut imx_terminal_config = hal::serial::FullConfig::default().baudrate(256_000.bps());
    use hal::serial::SerialExt;
    let mut imx_serial = device.USART2.usart(imx_terminal_tx,
                                         imx_terminal_rx,
                                         imx_terminal_config,
                                         &mut rcc).unwrap();
    let (mut usart2_dma_rx_buffer_p, usart2_dma_rx_buffer_c) = usart2_dma_rx_buffer.try_split().unwrap();
    let (mut usart2_dma_rcx, usart2_dma_mem_addr) =
        crate::tasks::dma::DmaRxContext::new(
            usart2_dma_rx_buffer_p,
            unsafe { hal::stm32::USART2::ptr() as *const _ as *mut hal::stm32::usart1::RegisterBlock },
            unsafe { hal::stm32::DMA1::ptr() as *const _ as *mut hal::stm32::dma1::RegisterBlock },
            //5
        );
    unsafe {
        // Enable clocks for DMA1 & DMAMUX
        let rcc = &(*hal::stm32::RCC::ptr());
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit().dmamuxen().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().set_bit().dmamux1rst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().clear_bit().dmamux1rst().clear_bit());
        let dma1 = &(*hal::stm32::DMA1::ptr());
        let dmamux = &(*hal::stm32::DMAMUX::ptr());
        let usart2 = &(*hal::stm32::USART2::ptr());
        // Configure DMA1 Stream 1
        dma1.ccr1.modify(|_, w| w.en().clear_bit());
        compiler_fence(Ordering::Acquire);
        dma1.ccr1.write(|w| w.bits(0));
        dma1.cndtr1.write(|w| w.bits(0));
        dma1.cpar1.write(|w| w.bits(0));
        dma1.cmar1.write(|w| w.bits(0));
        // Clear transfer complete flag & enable DMA receive request
        usart2.icr.write(|w| w.tccf().set_bit());
        usart2.cr3.modify(|_, w| w.dmar().set_bit());
        // Configure addresses and transfer size
        dma1.cndtr1.write(|w| w.bits(usart2_dma_rcx.buf_size() as u32)); // number of data items (bytes in this case) (bip buffer is power of 2 + 1)
        let usart2_rdr_addr: u32 = usart2 as *const _ as u32 + 0x24;
        dma1.cpar1.write(|w| w.bits(usart2_rdr_addr));
        dma1.cmar1.write(|w| w.bits(usart2_dma_mem_addr)); // memory address
        // Configure DMA1 Stream 1
        dma1.ccr1.modify(|_, w| w
            .teie().clear_bit()// 0: disabled, 1: enabled - transfer error interrupt enable
            .htie().set_bit() // half transfer interrupt enable
            .tcie().set_bit() // transfer complete interrupt enable
            .dir().clear_bit()      // 0: read from peripheral, 1: read from memory
            .circ().set_bit() // circular mode, 0: disabled, 1: enabled
            .pinc().clear_bit() // peripheral increment mode, 0: disabled, 1: enabled
            .psize().bits(0b00) // peripheral size, 00: 8 bits, 01: 16 bits, 10: 32 bits, 11: reserved
            .minc().set_bit()  // memory increment mode
            .msize().bits(0b00) // memory size
            .pl().bits(0b00) // priority
            .mem2mem().clear_bit() // 0: disabled, 1: enabled
        );
        // Configure DMAMUX
        dmamux.c0cr.modify(|_, w| w
            .spol().bits(0b00)
            .se().clear_bit()
            .ege().clear_bit()
            .soie().clear_bit()
            .dmareq_id().bits(26) // USART1_RX: 24, USART1_TX: 25, USART2_RX: 26, USART2_TX: 27, USART3_RX: 28, USART3_TX: 29, UART4_RX: 30, UART4_TX: 31, UART5_RX: 32, UART5_TX: 33, LPUART1_RX: 34, LPUART1_TX: 35
        );
        dmamux.rg0cr.modify(|_, w| w.gpol().bits(0b00).ge().set_bit());
        // Enable DMA1 Stream 1
        dma1.ccr1.modify(|_, w| w.en().set_bit());
        compiler_fence(Ordering::Release);
    }
    // Enable Idle IRQ
    imx_serial.listen(hal::serial::Event::Idle);
    let (lidar_frame_p, lidar_frame_c) = lidar_bbuffer.try_split_framed().unwrap();
    let lidar = crate::rplidar::RpLidar::new(lidar_frame_p);
    #[cfg(feature = "br")]
    cx.spawn.lidar(crate::rplidar::TaskEvent::PeriodicCheck).ok();

    cfg_if::cfg_if! {
            if #[cfg(feature = "master")] {
                crate::init::LateResources {
                    watchdog: iwdg,
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    trace_pin,
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    event_state_data,
                    channels: crate::channels::Channels::new(),

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),

                    rtt_down_channel: rtt_channels.down.0,

                    can0,
                    can0_irq_statistics,
                    can0_receive_heap,
                    can0_ll_statistics,
                    can0_rx_routing_table,
                    can0_rx_routing_statistics,
                    can0_local_processing_heap,
                    can0_analyzer,

                    imx_serial,
                    usart2_dma_rcx,
                    usart2_c: usart2_dma_rx_buffer_c,

                    counter_deltas: crate::tasks::blinker::CounterDeltas::default(),

                    #[cfg(feature = "tof")]
                    vl53l1_multi
                }
            } else if #[cfg(any(feature = "tr", feature = "bl", feature = "br"))] {
                crate::init::LateResources {
                    watchdog: iwdg,
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    trace_pin,
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    event_state_data,
                    channels: crate::channels::Channels::new(),

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),

                    rtt_down_channel: rtt_channels.down.0,

                    can0,
                    can0_irq_statistics,
                    can0_receive_heap,
                    can0_ll_statistics,
                    can0_rx_routing_table,
                    can0_rx_routing_statistics,
                    can0_local_processing_heap,
                    can0_analyzer,

                    imx_serial,
                    usart2_dma_rcx,
                    usart2_c: usart2_dma_rx_buffer_c,
                    #[cfg(feature = "br")]
                    lidar,
                    #[cfg(feature = "br")]
                    lidar_frame_c,

                    counter_deltas: crate::tasks::blinker::CounterDeltas::default(),
                    #[cfg(feature = "tof")]
                    vl53l1_multi
                }
            } else if #[cfg(feature = "anchor")] {
                crate::init::LateResources {
                    watchdog: iwdg,
                    clocks,

                    radio: radio::Radio::new(dw1000, dw1000_irq, radio_commands_c),
                    trace_pin,
                    radio_commands: radio_commands_p,
                    scheduler: Scheduler::new(),
                    event_state_data,
                    channels: crate::channels::Channels::new(),

                    led_blinky,
                    idle_counter: core::num::Wrapping(0u32),

                    rtt_down_channel: rtt_channels.down.0,

                    can0,
                    can0_irq_statistics,
                    can0_receive_heap,
                    can0_ll_statistics,
                    can0_rx_routing_table,
                    can0_rx_routing_statistics,
                    can0_local_processing_heap,
                    can0_analyzer,

                    imx_serial,

                    counter_deltas: crate::tasks::blinker::CounterDeltas::default(),
                }
            }
        }
}

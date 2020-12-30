use crate::board::hal;
use crate::color;
use crate::config;
use crate::radio;

use crate::hal::spi::Spi;
use bbqueue::BBBuffer;
use cfg_if::cfg_if;
use cortex_m::peripheral::DWT;
use dw1000::DW1000;
use embedded_hal::digital::v2::OutputPin;
use radio::scheduler::Scheduler;
use radio::types::Event;
use radio::types::RadioConfig;
use rtt_target::{rprint, rprintln, rtt_init_print};

use crate::radio::types::{Pong, ReadyRadio};
use core::sync::atomic::{compiler_fence, Ordering};
use dw1000::hl::SendTime;
use dw1000::mac::{Address, PanId, ShortAddress};
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::spi::FullDuplex;
use stm32g4xx_hal::gpio::SignalEdge;

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
    #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))]
    {
        let mut rcc = rcc.freeze(stm32g4xx_hal::rcc::Config::default());
        let clocks = rcc.clocks;
    }
    rprintln!("Clocks: {:#?}", clocks);
    let mut syscfg = device.SYSCFG;
    let mut exti = device.EXTI;

    use hal::gpio::GpioExt;
    #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))]
    {
        let mut gpioa = device.GPIOA.split(&mut rcc);
        let mut gpiob = device.GPIOB.split(&mut rcc);
        let mut gpioc = device.GPIOC.split(&mut rcc);
        let mut gpiod = device.GPIOD.split(&mut rcc);
        let mut gpiof = device.GPIOF.split(&mut rcc);
    }

    cfg_if! {
        if #[cfg(feature = "gcharger-board")] {
            let mut led_blinky = gpiob.pb15.into_push_pull_output();
        } else if #[cfg(feature = "gcarrier-board")] {
            let mut led_blinky = gpiob.pb1.into_push_pull_output();
        }
    }
    led_blinky.set_low().ok();

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
    use hal::can;
    let mut can_ctrl = CanController::new(ClockSource::Pllq, &mut rcc, &mut core.DWT).unwrap();
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
            can_regs
                .ile
                .write(|w| w.eint0().set_bit().eint1().clear_bit()); // enable irq0 line
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

    // DW1000
    let dw1000_spi_freq = 1.mhz();
    let dw1000_spi_freq_hi = 18.mhz();

    cfg_if! {
        if #[cfg(feature = "gcharger-board")] {
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
                rprintln!(
                    "DW1000: ok, model:{} ver:{} rev:{}",
                    info.model,
                    info.version,
                    info.revision
                );
            } else {
                rprintln!(
                    "{}DW1000 SPI communication error: IncorrectTag{}",
                    color::RED,
                    color::DEFAULT
                );
                panic!("DW1000 SPI broken");
            }
        }
        Err(e) => {
            rprintln!(
                "{}DW1000 SPI communication error: {:?}{}",
                color::RED,
                e,
                color::DEFAULT
            );
            panic!("DW1000 SPI broken");
        }
    }
    let dw1000 = dw1000.init(dw1000::configs::MaximumFrameLength::Decawave1023);
    let mut dw1000 = match dw1000 {
        Ok(dw1000) => dw1000,
        Err(e) => {
            panic!("DW1000 init error {:?}", e);
        }
    };
    for _ in 0..10 {
        let sys_status = dw1000.ll().sys_status().read().unwrap();
        if sys_status.clkpll_ll() == 0b0 {
            rprint!("SPI speed bump to: {}MHz, ", dw1000_spi_freq_hi.0);
            cfg_if! {
                if #[cfg(feature = "gcharger-board")] {
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
            #[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))]
            {
                rprintln!(
                    "done, actual ratio:{} baud:{}",
                    ratio,
                    clocks.apb1_clk.0 / ratio
                );
            }
            break;
        } else {
            rprintln!("clkpll_ll = 1, resetting");
            busywait!(ms_alt, clocks, 10);
        }
    }
    dw1000
        .set_address(config::PAN_ID, config::UWB_ADDR)
        .unwrap();
    dw1000
        .configure_leds(false, false, true, true, 1, true, false, true)
        .unwrap();
    //dw1000.set_antenna_delay(16456, 16300).expect("Failed to set antenna delay");
    //dw1000.set_antenna_delay(16147, 16166).expect("Failed to set antenna delay");
    dw1000
        .set_antenna_delay(16128, 16145)
        .expect("Failed to set antenna delay");
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

    let imx_terminal_tx = gpioc.pc4;
    let imx_terminal_rx = gpioc.pc5;
    use hal::time::Bps;
    let imx_terminal_config = hal::serial::Config::default().baudrate(115_200.bps());
    use hal::serial::SerialExt;
    let imx_serial = hal::serial::Serial::usart1(
        device.USART1,
        imx_terminal_tx,
        imx_terminal_rx,
        imx_terminal_config,
        &mut rcc,
    )
    .unwrap();

    cfg_if::cfg_if! {
        if #[cfg(feature = "master")] {
            crate::init::LateResources {
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
        } else if #[cfg(any(feature = "tr", feature = "bl", feature = "br"))] {
            crate::init::LateResources {
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
        } else if #[cfg(feature = "anchor")] {
            crate::init::LateResources {
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

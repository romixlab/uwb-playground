#![no_main]
#![no_std]

#[macro_use]
mod util;
mod board;
mod panic_handler;
mod radio;
mod channels;
mod config;
mod codec;
mod units;
mod tasks;
mod motion;
mod rplidar;
mod color;

use board::hal;
use core::num::Wrapping;
use rtic::{app};
use hal::rcc::Clocks;
use bbqueue::{BBBuffer, ConstBBBuffer};
use tasks::usart::{
    Usart1WorkerEvent,
    Usart2WorkerEvent
};

#[app(device = crate::board::hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,

        radio: radio::Radio,
        radio_commands: radio::types::CommandQueueP,
        scheduler: radio::scheduler::Scheduler,
        channels: channels::Channels,
        event_state_data: tasks::radio::EventStateData,

        // USART1 - tl/tr/bl/br <-> vesc.
        usart1_coder: codec::Usart1Coder,
        usart1_dma_tcx: config::Usart1DmaTxContext,
        usart1_dma_rcx: config::Usart1DmaRxContext,
        usart1_decoder: codec::Usart1Decoder,

        lift_serial: config::LiftSerial,

        // USART2 - master <-> ros, br <-> lidar.
        #[cfg(feature = "master")]
        usart2_coder: codec::Usart2Coder,
        #[cfg(any(feature = "master", feature = "br"))]
        usart2_dma_tcx: config::Usart2DmaTxContext,
        #[cfg(any(feature = "master", feature = "br"))]
        usart2_dma_rcx: config::Usart2DmaRxContext,
        #[cfg(feature = "master")]
        usart2_decoder: codec::Usart2Decoder,

        #[cfg(feature = "master")]
        mecanum_wheels: motion::MecanumWheels,
        #[cfg(feature = "slave")]
        wheel: motion::MCData,

        led_blinky: config::LedBlinkyPin,

        idle_counter: Wrapping<u32>,
        exti: hal::stm32::EXTI,

        #[cfg(feature = "br")]
        lidar: rplidar::RpLidar,
        #[cfg(feature = "master")]
        lidar_frame_c: rplidar::LidarBBufferC,
        #[cfg(feature = "br")]
        usart2_c: config::Usart2DmaRxBufferC,
        #[cfg(feature = "br")]
        usart2_p: config::Usart2DmaTxBufferP,

        #[cfg(feature = "master")]
        motion_channel_p: motion::ChannelP,
        #[cfg(feature = "slave")]
        motion_channel_c: motion::ChannelC,
        #[cfg(feature = "master")]
        motion_telemetry_c: motion::TelemetryChannelC,
        #[cfg(feature = "master")]
        local_motion_telemetry_p: motion::TelemetryLocalChannelP, // receive from mc -> C in arbiter
        #[cfg(feature = "slave")]
        motion_telemetry_p: motion::TelemetryChannelP,
}

    #[init(
        schedule = [],
        spawn = [
            radio_event,
            blinker,
            usart2_worker
        ]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RADIO_COMMANDS_QUEUE: radio::types::CommandQueue = heapless::spsc::Queue(heapless::i::Queue::new());
        static mut LIDAR_BBUFFER: rplidar::LidarBBuffer = BBBuffer(ConstBBBuffer::new());
        static mut USART1_DMA_RX_BUFFER: config::Usart1DmaRxBuffer = BBBuffer(ConstBBBuffer::new());
        static mut USART1_DMA_TX_BUFFER: config::Usart1DmaTxBuffer = BBBuffer(ConstBBBuffer::new());
        static mut USART2_DMA_RX_BUFFER: config::Usart2DmaRxBuffer = BBBuffer(ConstBBBuffer::new());
        static mut USART2_DMA_TX_BUFFER: config::Usart2DmaTxBuffer = BBBuffer(ConstBBBuffer::new());
        static mut MOTION_CHANNEL: motion::Channel = heapless::spsc::Queue(heapless::i::Queue::new());
        static mut TELEMETRY_CHANNEL: motion::TelemetryChannel = heapless::spsc::Queue(heapless::i::Queue::new());
        static mut LOCAL_TELEMETRY_CHANNEL: motion::TelemetryLocalChannel = heapless::spsc::Queue(heapless::i::Queue::new());
        tasks::init::init(
            cx,
            RADIO_COMMANDS_QUEUE,
            LIDAR_BBUFFER,
            USART1_DMA_RX_BUFFER,
            USART1_DMA_TX_BUFFER,
            USART2_DMA_RX_BUFFER,
            USART2_DMA_TX_BUFFER,
            MOTION_CHANNEL,
            TELEMETRY_CHANNEL,
            LOCAL_TELEMETRY_CHANNEL
        )
    }

    #[idle(
        resources = [
            idle_counter,
            mecanum_wheels,
        ]
    )]
    fn idle(cx: idle::Context) -> ! {
        tasks::idle::idle(cx);
    }

    #[task(
        resources = [
            &clocks,
            led_blinky,
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context) {
        static mut LED_STATE: bool = false;
        tasks::blinker::blinker(cx, LED_STATE);
    }

    #[task(
        binds = EXTI0,
        priority = 3,
        resources = [
            &clocks,
            radio,
            channels,
            scheduler,
            idle_counter,
        ],
        spawn = [radio_event],
        schedule = [radio_event]
    )]
    fn radio_irq(cx: radio_irq::Context) {
        static mut BUFFER: [u8; 1024] = [0u8; 1024];
        tasks::radio::radio_irq(cx, BUFFER);
    }

    #[task(
        priority = 2,
        capacity = 24,
        resources = [
            &clocks,
            radio,
            radio_commands,
            event_state_data,
            wheel,
            mecanum_wheels,
        ],
        spawn = [
            motor_control,
            usart2_worker,
        ],
        schedule = [
            radio_event,
        ]
    )]
    fn radio_event(cx: radio_event::Context, e: radio::types::Event) {
        tasks::radio::radio_event(cx, e);
    }

    #[task(
        priority = 4,

    )]
    fn usart1_worker(cx: usart1_worker::Context, e: Usart1WorkerEvent) {
        tasks::usart::usart1_worker(cx, e);
    }

    #[task(
        binds = USART1,
        priority = 4,
        resources = [usart1_dma_rcx],
        spawn = [usart1_worker]
    )]
    fn usart1_irq(cx: usart1_irq::Context) {
        let spawn = cx.spawn;
        cx.resources.usart1_dma_rcx.handle_usart_irq(|| {
            spawn.usart1_worker(Usart1WorkerEvent::Rx).ok();
        });
    }

    #[task(
        binds = DMA2_STREAM5,
        priority = 4,
        resources = [usart1_dma_rcx],
        spawn = [usart1_worker]
    )]
    fn usart1_dma_rx_irq(cx: usart1_dma_rx_irq::Context) {
        let spawn = cx.spawn;
        cx.resources.usart1_dma_rcx.handle_dma_rx_irq(|| {
            spawn.usart1_worker(Usart1WorkerEvent::Rx).ok();
        });
    }

    #[task(
    binds = DMA2_STREAM7,
    priority = 4,
    resources = [usart1_dma_tcx]
    )]
    fn usart1_dma_tx_irq(cx: usart1_dma_tx_irq::Context) {
        cx.resources.usart1_dma_tcx.handle_dma_tx_irq();
    }

    #[task(
        priority = 4,
        resources = [
            usart2_coder,
            usart2_decoder,
            motion_channel_p,
            lidar,
            usart2_p,
            usart2_c,
        ],
        spawn = [
            motor_control,
            lift_control,
        ]
    )]
    fn usart2_worker(cx: usart2_worker::Context, e: Usart2WorkerEvent) {
        tasks::usart::usart2_worker(cx, e);
    }

    #[task(
        binds = USART2,
        priority = 4,
        resources = [usart2_dma_rcx],
        spawn = [usart2_worker]
    )]
    fn usart2_irq(cx: usart2_irq::Context) {
        let spawn = cx.spawn;
        cx.resources.usart2_dma_rcx.handle_usart_irq(|| {
            spawn.usart2_worker(Usart2WorkerEvent::Rx).ok();
        });
    }

    #[task(
        binds = DMA1_STREAM5,
        priority = 4,
        resources = [usart2_dma_rcx],
        spawn = [usart2_worker]
    )]
    #[cfg(any(feature = "master", feature = "br"))]
    fn usart2_dma_rx_irq(cx: usart2_dma_rx_irq::Context) {
        let spawn = cx.spawn;
        cx.resources.usart2_dma_rcx.handle_dma_rx_irq(|| {
            spawn.usart2_worker(Usart2WorkerEvent::Rx).ok();
        });
    }

    #[task(
        binds = DMA1_STREAM6,
        priority = 4,
        resources = [usart2_dma_tcx]
    )]
    fn usart2_dma_tx_irq(cx: usart2_dma_tx_irq::Context) {
        cx.resources.usart2_dma_tcx.handle_dma_tx_irq();
    }

    #[task(
        priority = 4,
        capacity = 4,
        resources = [
            &clocks,
            mecanum_wheels,
            wheel,
            usart1_coder,
        ],
        schedule = [motor_control],
        spawn = [motor_control]
    )]
    fn motor_control(cx: motor_control::Context, e: motion::MotorControlEvent) {
        // static mut LAST_COMMAND_INSTANT: Option<rtic::cyccnt::Instant> = None;
        // static mut STOPPED: bool = false;
        tasks::motion::motor_control(cx, e);
    }

    #[task(
        binds = EXTI4,
        priority = 3,
        resources = [
            motion_channel_c,
            channels,
        ],
        spawn = [
            motor_control,
            usart2_worker,
        ]
    )]
    fn channel_event(cx: channel_event::Context) {
        tasks::channel_event::channel_event(cx);
    }

    #[task(
        priority = 2,
        capacity = 4,
        resources = [
            &clocks,
            lift_serial,
        ],
        schedule = [
            lift_control,
        ]
    )]
    fn lift_control(cx: lift_control::Context, command: tasks::lift::LiftControlCommand) {
        static mut TOKENS: u16 = 0;
        static mut CURRENT_SYMBOL: u8 = 0;
        tasks::lift::lift_control(cx, command, TOKENS, CURRENT_SYMBOL);
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
        fn EXTI9_5();
    }
};

#![no_main]
#![no_std]

#[macro_use]
mod util;
mod board;
mod panic_handler;
mod radio;
mod channels;
mod config;
mod crc_framer;
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

#[app(device = crate::board::hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        clocks: Clocks,

        radio: radio::Radio,
        radio_commands: radio::types::CommandQueueP,
        scheduler: radio::scheduler::Scheduler,
        channels: channels::Channels,
        event_state_data: tasks::radio::EventStateData,

        vesc_serial: config::VescSerial,
        vesc_framer: crc_framer::CrcFramerDe<generic_array::typenum::consts::U512>,
        vesc_bbbuffer_p: config::VescBBBufferP,
        vesc_bbbuffer_c: config::VescBBBufferC,

        lift_serial: config::LiftSerial,

        ctrl_serial: config::CtrlSerial,
        ctrl_framer: crc_framer::CrcFramerDe<generic_array::typenum::consts::U512>,
        ctrl_bbbuffer_p: config::CtrlBBBufferP,
        ctrl_bbbuffer_c: config::CtrlBBBufferC,

        #[cfg(feature = "master")]
        mecanum_wheels: motion::MecanumWheels,
        #[cfg(feature = "slave")]
        wheel: motion::MCData,

        led_blinky: config::LedBlinkyPin,

        idle_counter: Wrapping<u32>,
        exti: hal::stm32::EXTI,

        #[cfg(feature = "br")]
        lidar: rplidar::RpLidar,
        #[cfg(feature = "br")]
        lidar_queue_p: rplidar::LidarQueueP,
        #[cfg(feature = "br")]
        lidar_queue_c: rplidar::LidarQueueC,
    }

    #[init(
        schedule = [],
        spawn = [
            radio_event,
            blinker,
            ctrl_link_control
        ]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RADIO_COMMANDS_QUEUE: radio::types::CommandQueue = heapless::spsc::Queue(heapless::i::Queue::new());
        static mut VESC_BBBUFFER: BBBuffer<config::VescBBBufferSize> = BBBuffer(ConstBBBuffer::new());
        static mut CTRL_BBBUFFER: BBBuffer<config::CtrlBBBufferSize> = BBBuffer(ConstBBBuffer::new());
        static mut LIDAR_QUEUE: rplidar::LidarQueue = heapless::spsc::Queue(heapless::i::Queue::new());
        tasks::init::init(
            cx,
            RADIO_COMMANDS_QUEUE,
            VESC_BBBUFFER,
            CTRL_BBBUFFER,
            LIDAR_QUEUE
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
        priority = 6,
        resources = [
            &clocks,
            radio,
            channels,
            scheduler,
            idle_counter,
            exti,
        ],
        spawn = [
            radio_event,
        ],
        schedule = [
            radio_event,
        ]
    )]
    fn radio_irq(cx: radio_irq::Context) {
        static mut BUFFER: [u8; 1024] = [0u8; 1024];
        tasks::radio::radio_irq(cx, BUFFER);
    }

    #[task(
        priority = 4,
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
            ctrl_link_control,
        ],
        schedule = [
            radio_event,
        ]
    )]
    fn radio_event(cx: radio_event::Context, e: radio::types::Event) {
        tasks::radio::radio_event(cx, e);
    }

    #[task(
        priority = 2,
        capacity = 4,
        resources = [
            &clocks,
            mecanum_wheels,
            ctrl_bbbuffer_p,
            lidar,
            lidar_queue_c,
        ],
        schedule = [
            ctrl_link_control,
        ]
    )]
    fn ctrl_link_control(cx: ctrl_link_control::Context) {
        tasks::ctrl_link::ctrl_link_control(cx);
    }

    #[task(
        binds = USART2,
        priority = 3,
        resources = [
            ctrl_serial,
            ctrl_framer,
            ctrl_bbbuffer_p,
            ctrl_bbbuffer_c,
            lidar,
            lidar_queue_p,
        ],
        spawn = [
            motor_control,
            lift_control,
        ]
    )]
    fn ctrl_serial_irq(cx: ctrl_serial_irq::Context) {
        static mut SENDING: Option<bbqueue::GrantR<'static, config::CtrlBBBufferSize>> = None;
        static mut SENDING_IDX: usize = 0;
        tasks::ctrl_link::ctrl_serial_irq(cx, SENDING, SENDING_IDX);
    }

    #[task(
        priority = 2,
        capacity = 4,
        resources = [&clocks, mecanum_wheels, wheel, vesc_bbbuffer_p],
        schedule = [motor_control],
        spawn = [motor_control]
    )]
    fn motor_control(cx: motor_control::Context, e: motion::MotorControlEvent) {
        static mut LAST_COMMAND_INSTANT: Option<rtic::cyccnt::Instant> = None;
        static mut STOPPED: bool = false;
        tasks::motion::motor_control(cx, e, LAST_COMMAND_INSTANT, STOPPED);
    }

    #[task(
        binds = USART1,
        priority = 3,
        resources = [
            vesc_serial,
            vesc_framer,
            vesc_bbbuffer_c,
            mecanum_wheels,
            wheel
        ]
    )]
    fn vesc_serial_irq(cx: vesc_serial_irq::Context) {
        static mut SENDING: Option<bbqueue::GrantR<'static, config::VescBBBufferSize>> = None;
        static mut SENDING_IDX: usize = 0;
        tasks::motion::vesc_serial_irq(cx, SENDING, SENDING_IDX);
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

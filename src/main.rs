#![no_main]
#![no_std]

#[macro_use]
mod util;
mod board;
mod panic_handler;
mod radio;
mod channels;
mod config;
//mod codec;
mod units;
mod tasks;
//mod motion;
//mod rplidar;
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
        channels: channels::Channels,
        scheduler: radio::scheduler::Scheduler,
        event_state_data: tasks::radio::EventStateData,
        led_blinky: config::LedBlinkyPin,
        idle_counter: Wrapping<u32>,
        rtt_down_channel: rtt_target::DownChannel
    }

    #[init(
        schedule = [],
        spawn = [
            radio_event,
            blinker,
        ]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RADIO_COMMANDS_QUEUE: radio::types::CommandQueue = heapless::spsc::Queue(heapless::i::Queue::new());
        tasks::init::init(
            cx,
            RADIO_COMMANDS_QUEUE,
        )
    }

    #[idle(
        resources = [
            idle_counter,
            rtt_down_channel,
            radio_commands
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
        binds = EXTI15_10,
        priority = 3,
        resources = [
            &clocks,
            radio,
            scheduler,
            idle_counter,
            channels,
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
        ],
        spawn = [
        ],
        schedule = [
            radio_event,
        ]
    )]
    fn radio_event(cx: radio_event::Context, e: radio::types::Event) {
        tasks::radio::radio_event(cx, e);
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
        fn EXTI9_5();
    }
};

use cortex_m_rt::exception;
#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // prints the exception frame as a panic message
    panic!("{:#?}", ef);
}
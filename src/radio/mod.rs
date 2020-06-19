pub mod message;
pub mod state_machine;

use dw1000::{DW1000, Ready, Receiving, Sending};
use crate::config;
use crate::board::hal;

use heapless::spsc::{Queue, Producer, Consumer};
use heapless::consts::*;

pub type Dw1000Spi = hal::spi::Spi<hal::stm32::SPI1,
    (config::Dw1000Clk, config::Dw1000Miso, config::Dw1000Mosi)>;

pub type ReadyRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Ready>;
pub type SendingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Sending>;
pub type ReceivingRadio = DW1000<Dw1000Spi, config::Dw1000Cs, Receiving>;

pub enum RadioState {
    Ready(Option<ReadyRadio>),

    #[cfg(feature = "master")]
    GTSStartSending(Option<SendingRadio>),
    #[cfg(feature = "master")]
    GTSAnswersReceiving((Option<ReceivingRadio>, u8)),

    #[cfg(feature = "slave")]
    GTSStartWaiting(Option<ReceivingRadio>),
    #[cfg(feature = "slave")]
    GTSAnswerSending(Option<SendingRadio>),
}

pub enum Command {
    #[cfg(feature = "master")]
    GTSStart(message::GTSStart)
}

pub type CommandQueue = Queue<Command, U8>;
pub type CommandQueueP = Producer<'static, Command, U8>;
pub type CommandQueueC = Consumer<'static, Command, U8>;
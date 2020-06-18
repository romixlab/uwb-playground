pub mod message;
pub mod state_machine;

use dw1000::{DW1000, Ready, Receiving, Sending};
use crate::config;
use stm32f4xx_hal as hal;

pub enum DW1000State<SPI, CS> {
    Ready(Option<DW1000<SPI, CS, Ready>>),
    Sending(Option<DW1000<SPI, CS, Sending>>),
    Receiving(Option<DW1000<SPI, CS, Receiving>>),
}

pub type Dw1000Spi = hal::spi::Spi<hal::stm32::SPI1,
    (config::Dw1000Clk, config::Dw1000Miso, config::Dw1000Mosi)>;
pub type UWBRadio = DW1000State<Dw1000Spi, config::Dw1000Cs>;

#[derive(Debug, PartialEq)]
pub enum RadioState {
    Idle,
    #[cfg(feature = "master")]
    GTSStart,
    #[cfg(feature = "slave")]
    GTSStartWait,
    #[cfg(feature = "slave")]
    GTSWindowWait,
    #[cfg(feature = "master")]
    GTSAnswer(u8),
}

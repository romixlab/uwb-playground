#[cfg(feature = "pozyx-board")]
pub use stm32f4xx_hal as hal;

#[cfg(feature = "dragonfly-board")]
pub use stm32l4xx_hal as hal;
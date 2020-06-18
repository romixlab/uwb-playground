use stm32f4xx_hal::gpio::{PushPull, Output, Alternate, AF5};
use stm32f4xx_hal::gpio::{gpioa::{PA4, PA5, PA6, PA7}};
use stm32f4xx_hal::stm32::Interrupt;

pub type Dw1000Clk = PA5<Alternate<AF5>>;
pub type Dw1000Miso = PA6<Alternate<AF5>>;
pub type Dw1000Mosi = PA7<Alternate<AF5>>;
pub type Dw1000Cs = PA4<Output<PushPull>>;

/// Blink LED with specified period (alive indicator).
pub const BLINK_PERIOD_MS: u32 = 500;

/// Ignore IRQ and check state anyway with specified period.
/// To ensure that endless lockup won't happen.
#[cfg(feature = "slave")]
pub const DW1000_CHECK_PERIOD_MS: u32 = 1000;

/// Which EXTI line is used for DW1000 interrupt.
/// Ensure that radio_irq task is coherent with this!
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI0;

/// Period for synchronous data exchange (guaranteed time slots (GTS) with slaves).
#[cfg(feature = "master")]
pub const GTS_PERIOD_MS: u32 = 20;

/// Allocate at least this amount of GTS and signal a failure (to all slaves and to uplink)
/// if one or more is missing for >= THRESH
#[cfg(feature = "master")]
pub const REQUIRED_SLAVE_COUNT: u8 = 3;
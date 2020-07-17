use crate::board::hal;
use crate::units::{
    MilliSeconds,
    ms
};
use hal::gpio::{PushPull, Output, Alternate, Input, PullDown};

#[cfg(feature = "pozyx-board")]
use hal::gpio::{AF5, gpioa::{PA4, PA5, PA6, PA7}};

#[cfg(feature = "pozyx-board")]
pub type Dw1000Clk = PA5<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Miso = PA6<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Mosi = PA7<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Cs = PA4<Output<PushPull>>;

#[cfg(feature = "dragonfly-board")]
use hal::gpio::{AF5};
#[cfg(feature = "dragonfly-board")]
use hal::gpio::{gpioa::{PA8}, gpiob::{PB3, PB4, PB5}};

#[cfg(feature = "dragonfly-board")]
pub type Dw1000Clk = PB3<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Miso = PB4<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Mosi = PB5<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Cs = PA8<Output<PushPull>>;

/// Blink LED with specified period (alive indicator).
pub const BLINK_PERIOD_MS: u32 = 500;

/// Ignore IRQ and check state anyway with specified period.
/// To ensure that endless lockup won't happen.
#[cfg(feature = "slave")]
pub const DW1000_CHECK_PERIOD: MilliSeconds = ms(1000);

use hal::stm32::Interrupt;
/// Which EXTI line is used for DW1000 interrupt.
/// Ensure that radio_irq task is coherent with this!
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI0;

/// Period for synchronous data exchange (guaranteed time slots (GTS) with slaves).
pub const GTS_PERIOD: MilliSeconds = ms(50);

use dw1000::mac::{PanId, ShortAddress};
pub const PAN_ID: PanId = PanId(0x666);

/// Allocate at least this amount of GTS and signal a failure (to all slaves and to uplink)
/// if one or more is missing for >= THRESH
pub const REQUIRED_SLAVE_COUNT: u8 = 3;

/// Maximum number of nodes in a PAN
pub type TotalNodeCount = typenum::consts::U5;

#[cfg(feature = "master")]
pub const UWB_ADDR: ShortAddress = ShortAddress(0x999);

#[cfg(any(feature = "master", feature = "devnode", feature = "tr"))]
pub const TR_UWB_ADDR: ShortAddress = ShortAddress(0xaa_00); // 43520

#[cfg(any(feature = "master", feature = "devnode", feature = "bl"))]
pub const BL_UWB_ADDR: ShortAddress = ShortAddress(0xaa_01); // 43521

#[cfg(any(feature = "master", feature = "devnode", feature = "br"))]
pub const BR_UWB_ADDR: ShortAddress = ShortAddress(0xaa_02); // 43522

#[cfg(feature = "devnode")]
pub const DEV_UWB_ADDR: ShortAddress = ShortAddress(0x777);

#[cfg(feature = "tr")]
pub const SLAVE_ID: usize = 0;
#[cfg(feature = "tr")]
pub const UWB_ADDR: ShortAddress = TR_UWB_ADDR;

#[cfg(feature = "bl")]
pub const SLAVE_ID: usize = 1;
#[cfg(feature = "bl")]
pub const UWB_ADDR: ShortAddress = BL_UWB_ADDR;

#[cfg(feature = "br")]
pub const SLAVE_ID: usize = 2;
#[cfg(feature = "br")]
pub const UWB_ADDR: ShortAddress = BR_UWB_ADDR;

#[cfg(feature = "devnode")]
pub const SLAVE_ID: usize = REQUIRED_SLAVE_COUNT as usize - 1;
#[cfg(feature = "devnode")]
pub const UWB_ADDR: ShortAddress = DEV_UWB_ADDR;

pub mod motor_control {
    /// How often timing checks are performed.
    pub const TIMING_CHECK_INTERVAL_MS: u32 = 100;
    /// Turn off all motors if no move commands received for this amount of time.
    pub const STOP_TIMEOUT_MS: u32 = 500;
}

#[cfg(feature = "pozyx-board")]
use hal::gpio::{gpioa::{PA0, PA1}, gpiob::{PB5}};
#[cfg(feature = "pozyx-board")]
pub type LedBlinkyPin = PB5<Output<PushPull>>;
#[cfg(feature = "dragonfly-board")]
pub type LedBlinkyPin = PC10<Output<PushPull>>;

#[cfg(feature = "pozyx-board")]
pub type RadioIrqPin = PA0<Input<PullDown>>;
//type RadioIrqPin = PC4<Input<PullDown>>;
#[cfg(feature = "dragonfly-board")]
pub type RadioIrqPin = PC9<Input<PullDown>>;

#[cfg(feature = "pozyx-board")]
pub type RadioTracePin = PA1<Output<PushPull>>;
#[cfg(feature = "dragonfly-board")]
pub type RadioTracePin = PC8<Output<PushPull>>;

use hal::gpio::gpiob::{PB6, PB7};
use hal::gpio::AF7;

pub type VescTxPin = PB6<Alternate<AF7>>;
pub type VescRxPin = PB7<Alternate<AF7>>;
pub type VescSerial = hal::serial::Serial<hal::stm32::USART1, (VescTxPin, VescRxPin)>;
pub type VescBBBufferSize = generic_array::typenum::consts::U512;
pub type VescBBBufferP = bbqueue::Producer<'static, VescBBBufferSize>;
pub type VescBBBufferC = bbqueue::Consumer<'static, VescBBBufferSize>;
pub const VESC_IRQ_EXTI: Interrupt = Interrupt::USART1;
#[cfg(feature = "master")]
pub const VESC_RPM_ARRAY_FRAME_ID: u8 = 86;
#[cfg(feature = "master")]
pub const VESC_TACHO_ARRAY_FRAME_ID: u8 = 87;
pub const VESC_POWER_ARRAY_FRAME_ID: u8 = 91;
pub const VESC_SETRPM_FRAME_ID: u8 = 8;
pub const VESC_SETCURRENT_FRAME_ID: u8 = 6;
pub const VESC_REQUEST_VALUES_SELECTIVE_FRAME_ID: u8 = 50;
pub const VESC_LIDAR_FRAME_ID: u8 = 90;
#[cfg(feature = "master")]
pub const VESC_LIFT_FRAME_ID: u8 = 88;
#[cfg(feature = "master")]
pub const VESC_RESET_ALL: u8 = 89;
pub const VESC_REQUESTED_VALUES: u32 = (1 << 13) | (1 << 3) | (1 << 8); // tacho + i_in + v_in

use hal::gpio::gpioa::{PA2, PA3};
pub type CtrlTxPin = PA2<Alternate<AF7>>;
pub type CtrlRxPin = PA3<Alternate<AF7>>;
pub type CtrlSerial = hal::serial::Serial<hal::stm32::USART2, (CtrlTxPin, CtrlRxPin)>;
pub type CtrlBBBufferSize = generic_array::typenum::consts::U8192;
pub type CtrlBBBufferP = bbqueue::Producer<'static, CtrlBBBufferSize>;
pub type CtrlBBBufferC = bbqueue::Consumer<'static, CtrlBBBufferSize>;
#[cfg(any(feature = "master", feature = "devnode", feature = "br"))] // for lidar
pub const CTRL_IRQ_EXTI: Interrupt = Interrupt::USART2;
pub const CTRL_TX_DMA: Interrupt = Interrupt::DMA1_STREAM6;

pub const CHANNEL_EVENT_IRQ: Interrupt = Interrupt::EXTI4;

use hal::gpio::gpioc::{PC6};
use hal::gpio::AF8;
use stm32f4xx_hal::serial::NoRx;

pub type LiftTxPin = PC6<Alternate<AF8>>;
pub type LiftSerial = hal::serial::Serial<hal::stm32::USART6, (LiftTxPin, NoRx)>;
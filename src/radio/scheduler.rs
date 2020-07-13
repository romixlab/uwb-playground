use super::channelization::{
    Multiplex,
    ChannelId,
    LogicalDestination,
};
use super::types::{
    Window,
    WindowType,
    RadioConfig,
};
use dw1000::{
    configs::{
        UwbChannel,
        BitRate,
        PulseRepetitionFrequency,
    }
};
use crate::units::MicroSeconds;
use crate::config;

pub struct Scheduler {

}

impl Scheduler {
    #[cfg(feature = "master")]
    pub fn source_timeslots<M: Multiplex<Error = super::Error>>(mux: &mut M)
    {
        let mut window = Window {
            shift: MicroSeconds(0),
            window: MicroSeconds(940),
            window_type: WindowType::Uplink,
            radio_config: RadioConfig {
                channel: UwbChannel::Channel5,
                bitrate: BitRate::Kbps850,
                prf: PulseRepetitionFrequency::Mhz64
            }
        };
        let ctrl_ch = ChannelId::ctrl();
        // GTS
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::TR_UWB_ADDR), ctrl_ch);
        window.shift = MicroSeconds(1640);
        window.window = MicroSeconds(940);
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::BL_UWB_ADDR), ctrl_ch);
        window.shift = MicroSeconds(3280);
        window.window = MicroSeconds(940);
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::BR_UWB_ADDR), ctrl_ch);
        window.shift = MicroSeconds(4920);
        window.window = MicroSeconds(1980);
        let _ = mux.mux(&window, LogicalDestination::Implicit, ctrl_ch);
        // Dynamic
        window.radio_config.bitrate = BitRate::Kbps6800;

        window.shift = MicroSeconds(7600);
        window.window = MicroSeconds(4440);
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::TR_UWB_ADDR), ctrl_ch);
        window.shift = MicroSeconds(12750);
        window.window = MicroSeconds(4440);
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::BL_UWB_ADDR), ctrl_ch);
        window.shift = MicroSeconds(17900);
        window.window = MicroSeconds(5420);
        let _ = mux.mux(&window, LogicalDestination::Unicast(config::BR_UWB_ADDR), ctrl_ch);
    }

    /// Guaranteed time slot phase duration.
    /// (not implemented) May be 0 if no slots had been given to anyone.
    pub fn gts_phase_duration() -> MicroSeconds { MicroSeconds(4920) }

    /// Aloha slot duration. Anyone except master can send in this period.
    /// Randomize send time in this slot.
    /// Even better - use slotted Aloha if several messages with equal length need to be sent.
    pub fn aloha_phase_duration() -> MicroSeconds { MicroSeconds(1980) }

    /// Dynamic slots phase duration.
    /// (not implemented) May be 0 if no slots had been given to anyone.
    pub fn dyn_phase_duration() -> MicroSeconds { MicroSeconds(15700) }

    // pub fn ranging_phase_duration() -> MicroSeconds { MicroSeconds() }

    /// GTS phase should definitely end at this moment.
    pub fn gts_period_end() -> MicroSeconds {
        Self::gts_phase_duration() + Self::quarter_guard()
    }

    pub fn aloha_period_start() -> MicroSeconds {
        Self::gts_phase_duration() + Self::guard()
    }

    pub fn aloha_period_end() -> MicroSeconds {
        Self::aloha_period_start() + Self::quarter_guard()
    }

    pub fn dyn_period_start() -> MicroSeconds {
        Self::aloha_period_start() + Self::aloha_phase_duration() + Self::guard()
    }

    pub fn dyn_period_end() -> MicroSeconds {
        Self::dyn_period_start() + Self::quarter_guard()
    }

    pub fn guard() -> MicroSeconds { MicroSeconds(700) }
    pub fn half_guard() -> MicroSeconds { Self::guard() / 2 }
    pub fn quarter_guard() -> MicroSeconds { Self::guard() / 4 }
}
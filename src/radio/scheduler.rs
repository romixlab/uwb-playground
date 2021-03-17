use super::channelization::{
    Multiplex,
    ChannelId,
    LogicalDestination,
};
use super::types::{
    Slot,
    SlotType,
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
use core::iter::Filter;
use crate::radio::types::ReadyRadio;

pub struct Scheduler {
    #[cfg(feature = "master")]
    slots: [Slot; 10],
    #[cfg(feature = "master")]
    active: usize,
}

#[cfg(feature = "master")]
impl<'a> IntoIterator for &'a Scheduler {
    type Item = Slot;
    type IntoIter = SlotIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        SlotIterator {
            slots: &self.slots[..self.active],
            index: 0
        }
    }
}

#[cfg(feature = "master")]
pub struct SlotIterator<'a> {
    slots: &'a [Slot],
    index: usize
}

#[cfg(feature = "master")]
impl<'a> Iterator for SlotIterator<'a> {
    type Item = Slot;

    fn next(&mut self) -> Option<Self::Item> {
        let result = if self.index < self.slots.len() {
            Some(self.slots[self.index])
        } else {
            None
        };
        self.index += 1;
        result
    }
}

impl Scheduler {
    #[cfg(feature = "master")]
    pub fn new() -> Self {
        let mut slots = [Slot::default(); 10];
        let mut slot = Slot::default();
        // GTS
        slot.slot_type = SlotType::GtsUplink;
        slot.radio_config = RadioConfig::default();
        slot.radio_config.bitrate = BitRate::Kbps6800;

        slot.shift = MicroSeconds(0);
        slot.duration = MicroSeconds(1000);
        slots[0] = slot;

        slot.shift = MicroSeconds(2000);
        slot.duration = MicroSeconds(1000);
        slots[1] = slot;

        slot.shift = MicroSeconds(4000);
        slot.duration = MicroSeconds(2500);
        slots[2] = slot;

        slot.shift = MicroSeconds(7500);
        slot.duration = MicroSeconds(2500);
        slots[3] = slot;
        // Aloha
        // slot.slot_type = SlotType::Aloha;
        // slot.shift = Self::aloha_period_start();
        // slot.duration = MicroSeconds(1000);//Self::aloha_phase_duration();
        // slots[3] = slot;
        // Dynamic
        // slot.radio_config.bitrate = BitRate::Kbps6800;
        // slot.slot_type = SlotType::DynUplink;
        // slot.shift = MicroSeconds(7600);
        // slot.duration = MicroSeconds(1000);
        // slots[3] = slot;
        // slot.shift = MicroSeconds(12750);
        // slot.duration = MicroSeconds(1000);
        // slots[4] = slot;
        // slot.shift = MicroSeconds(17900);
        // slot.duration = MicroSeconds(5420);
        // slots[5] = slot;
        // Ranging
        // slot.radio_config.bitrate = BitRate::Kbps6800;
        // slot.slot_type = SlotType::Ranging;
        // slot.shift = Self::ranging_period_start();
        // slot.duration = Self::ranging_phase_duration();
        // slots[4] = slot;

        Scheduler {
            slots,
            active: 0
        }
    }

    #[cfg(feature = "slave")]
    pub fn new() -> Self {
        Scheduler {}
    }

    #[cfg(feature = "anchor")]
    pub fn new() -> Self {
        Scheduler {}
    }

    #[cfg(feature = "master")]
    pub fn source_timeslots<M: Multiplex<Error = super::Error>>(&self, mux: &mut M)
    {
        let ctrl_ch = ChannelId::ctrl();
        use SlotType::*;
        use LogicalDestination::*;

        // GTS
        let mut gt_slots = self.into_iter().filter(|s| s.slot_type == GtsUplink);
        let destinations = [
            Unicast(config::TR_UWB_ADDR),
            Unicast(config::BL_UWB_ADDR),
            Unicast(config::BR_UWB_ADDR),
            Unicast(config::BR_UWB_ADDR),
        ];
        for (s, d) in gt_slots.zip(destinations.iter()) {
            let _ = mux.mux(&s, *d, ctrl_ch);
        }

        // Aloha
        let aloha_slot = self.into_iter().find(|s| s.slot_type == Aloha);
        match aloha_slot {
            Some(s) => {
                let _ = mux.mux(&s, Implicit, ctrl_ch);
            },
            None => {}
        }

        // Dynamic
        let mut dyn_uplink_slots = self.into_iter().filter(|s| s.slot_type == DynUplink);
        for (s, d) in dyn_uplink_slots.zip(destinations.iter()) {
            let _ = mux.mux(&s, *d, ctrl_ch);
        }

        // Ranging
        let ranging_slot = self.into_iter().find(|s| s.slot_type == Ranging);
        match ranging_slot {
            Some(s) => {
                let _ = mux.mux(&s, Implicit, ctrl_ch);
            },
            None => {}
        }
    }

    #[cfg(feature = "master")]
    pub fn dyn_slots() {

    }

    /// Guaranteed time slot phase duration.
    /// (not implemented) May be 0 if no slots had been given to anyone.
    pub fn gts_phase_duration() -> MicroSeconds { MicroSeconds(14000) }

    /// Aloha slot duration. Anyone except master can send in this period.
    /// Randomize send time in this slot.
    /// Even better - use slotted Aloha if several messages with equal length need to be sent.
    pub fn aloha_phase_duration() -> MicroSeconds { MicroSeconds(0) }

    /// Dynamic slots phase duration.
    /// (not implemented) May be 0 if no slots had been given to anyone.
    pub fn dyn_phase_duration() -> MicroSeconds { MicroSeconds(0) }

    pub fn ranging_phase_duration() -> MicroSeconds { MicroSeconds(3000) }

    /// GTS phase should definitely end at this moment.
    pub fn gts_period_end() -> MicroSeconds {
        Self::gts_phase_duration() + Self::quarter_guard()
    }

    pub fn aloha_period_start() -> MicroSeconds {
        Self::gts_phase_duration() + Self::guard()
    }

    pub fn aloha_period_end() -> MicroSeconds {
        Self::aloha_period_start() + Self::aloha_phase_duration() + Self::quarter_guard()
    }

    pub fn dyn_period_start() -> MicroSeconds {
        Self::aloha_period_start() + Self::aloha_phase_duration() + Self::guard()
    }

    pub fn dyn_period_end() -> MicroSeconds {
        Self::dyn_period_start() + Self::dyn_phase_duration() + Self::quarter_guard()
    }

    pub fn ranging_period_start() -> MicroSeconds {
        //Self::dyn_period_start() + Self::dyn_phase_duration() + Self::guard()
        Self::gts_phase_duration() + Self::guard()
    }

    pub fn ranging_period_end() -> MicroSeconds {
        Self::ranging_period_start() + Self::ranging_phase_duration() + Self::quarter_guard()
    }

    pub fn guard() -> MicroSeconds { MicroSeconds(700) }
    pub fn half_guard() -> MicroSeconds { Self::guard() / 2 }
    pub fn quarter_guard() -> MicroSeconds { Self::guard() / 4 }
}

// pub struct Trainer {
//
// }
//
// impl Trainer {
//     pub fn train(ready_radio: ReadyRadio) ->
// }
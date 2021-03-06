use super::serdes::{
    Serialize,
    Deserialize,
    Buf,
    BufMut,
    MessageSpec,
};
use dw1000::configs::{
    UwbChannel,
    BitRate,
};
use super::types::{
    Slot,
    SlotType,
    RadioConfig,
};
use super::Error;
use crate::units::MicroSeconds;
use crate::radio::types::{Pong, DummyMessage};
use dw1000::ranging::{Ping as RangingPing, Request as RangingRequest, Response as RangingResponse};

fn channel_from_u8(n: u8) -> Option<UwbChannel> {
    use UwbChannel::*;
    match n {
        1 => { Some(Channel1) },
        2 => { Some(Channel2) },
        3 => { Some(Channel3) },
        4 => { Some(Channel4) },
        5 => { Some(Channel5) },
        7 => { Some(Channel7) },
        _ => None
    }
}

fn bitrate_from_u8(n: u8) -> Option<BitRate> {
    use BitRate::*;
    match n {
        0b00 => { Some(Kbps110) },
        0b01 => { Some(Kbps850) },
        0b10 => { Some(Kbps6800) },
        _ => None
    }
}

fn slot_type_from_u8(n: u8) -> Option<SlotType> {
    use SlotType::*;
    match n {
        0b000 => Some(GtsUplink),
        0b001 => Some(Downlink),
        0b010 => Some(Aloha),
        0b011 => Some(DynUplink),
        0b100 => Some(Ranging),
        _ => None
    }
}

impl MessageSpec for Slot {
    const ID: u8 = 0x70;
    const SIZE: usize = 7;
}

// &[u8] ID = 0xfd

impl MessageSpec for DummyMessage {
    const ID: u8 = 0xfe;
    const SIZE: usize = 1; // + len
}

impl MessageSpec for Pong {
    const ID: u8 = 0x30;
    const SIZE: usize = 1;
}

impl MessageSpec for RangingPing {
    const ID: u8 = 0xc0;
    const SIZE: usize = 6;
}

impl MessageSpec for RangingRequest {
    const ID: u8 = 0xc1;
    const SIZE: usize = 16;
}

impl MessageSpec for RangingResponse {
    const ID: u8 = 0xc2;
    const SIZE: usize = 21;
}

/// Check if ID can be readed, do additional checks and then eat ID from the buf.
/// `buf`, `id`, `additional_checks`.
macro_rules! eat_id {
    ($buf:ident, $id:expr, $additional_checks:tt) => {
        if $buf.remaining() >= 1 {
            if $buf.peek_u8() != $id {
                return Err(Error::WrongId);
            }
        } else {
            return Err(Error::Eof);
        }
        $additional_checks
        let _id = $buf.get_u8();
    };
}

/// Check if there is space available, do additional checks and put ID to buf.
/// `buf`, `id`, `size`, `additional_checks`.
macro_rules! put_id {
    ($buf:ident, $id:expr, $size:expr, $additional_checks:tt) => {
        if $buf.remaining() < $size {
            return Err(Error::NotEnoughSpace);
        }
        $additional_checks
        $buf.put_u8($id);
    };
}

impl Serialize for Pong {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE, {});
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for Pong {
    type Output = Pong;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID, {});
        Ok(Pong{})
    }
}

impl Serialize for Slot {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE, {
            if self.shift.0 >= core::u16::MAX as u32 || self.duration.0 >= core::u16::MAX as u32 {
                return Err(Error::WindowTooLong);
            }
        });
        let shift = self.shift.0 as u16;
        let window = self.duration.0 as u16;
        buf.put_u16(shift);
        buf.put_u16(window);
        let channel = self.radio_config.channel as u8;
        let bitrate = self.radio_config.bitrate as u8;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if self.radio_config.prf == Mhz16 { 0u8 } else { 1u8 };
        buf.put_u8((prf << 6) | (bitrate << 4) | channel); // bit 7 = reserved
        let slot_type = self.slot_type as u8;
        buf.put_u8(slot_type & 0b0000_0111); // bits 7:3 = reserved
        Ok(())
    }

    fn size_hint(&self) -> usize {
        Self::SIZE
    }
}

impl Serialize for &[u8] {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, 0xfd, self.len() + 1, {});
        buf.put_slice(self);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        self.len() + 1
    }
}

impl Deserialize for Slot {
    type Output = Slot;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID, {
            if buf.remaining() < Self::SIZE - 1 {
                return Err(Error::Eof);
            }
        });

        let shift = buf.get_u16();
        let window = buf.get_u16();
        let config = buf.get_u8();
        let channel = channel_from_u8(config & 0b0000_1111).ok_or(Error::WrongChannel)?;
        let bitrate = bitrate_from_u8((config & 0b0011_0000) >> 4).ok_or(Error::WrongBitrate)?;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if config & 0b0100_0000 == 0 { Mhz16 } else { Mhz64 };
        use SlotType::*;
        let slot_type = slot_type_from_u8(buf.get_u8() & 0b0000_0111).ok_or(Error::WrongSlotType)?;
        Ok(Slot {
            shift: MicroSeconds(shift as u32),
            duration: MicroSeconds(window as u32),
            slot_type,
            radio_config: RadioConfig {
                channel,
                bitrate,
                prf
            }
        })
    }
}

//
// Ranging
//
macro_rules! eat_instant {
    ($buf:ident) => {
        {
            let mut instant = [0u8; 8];
            instant[0..5].copy_from_slice($buf.slice_to(5));
            let instant = u64::from_le_bytes(instant);
            $buf.advance(5);
            dw1000::time::Instant::new(instant).expect("eat_instant")
        }
    };
}

macro_rules! eat_duration {
    ($buf:ident) => {
        {
            let mut duration = [0u8; 8];
            duration[0..5].copy_from_slice($buf.slice_to(5));
            let duration = u64::from_le_bytes(duration);
            $buf.advance(5);
            dw1000::time::Duration::new(duration).expect("eat_duration")
        }
    };
}

impl Serialize for RangingPing {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE, {});
        let ping_tx_time = self.ping_tx_time.value().to_le_bytes();
        buf.put_slice(&ping_tx_time[0..5]);
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for RangingPing {
    type Output = RangingPing;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID, {});
        let mut ping_tx_time = eat_instant!(buf);
        Ok(RangingPing{ ping_tx_time })
    }
}

impl Serialize for RangingRequest {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE, {});
        let ping_tx_time = self.ping_tx_time.value().to_le_bytes();
        let ping_reply_time = self.ping_reply_time.value().to_le_bytes();
        let request_tx_time = self.request_tx_time.value().to_le_bytes();
        buf.put_slice(&ping_tx_time[0..5]);
        buf.put_slice(&ping_reply_time[0..5]);
        buf.put_slice(&request_tx_time[0..5]);
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for RangingRequest {
    type Output = RangingRequest;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID, {});
        let mut ping_tx_time = eat_instant!(buf);
        let mut ping_reply_time = eat_duration!(buf);
        let mut request_tx_time = eat_instant!(buf);

        Ok(RangingRequest{ ping_tx_time, ping_reply_time, request_tx_time })
    }
}

impl Serialize for RangingResponse {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE, {});
        let ping_reply_time = self.ping_reply_time.value().to_le_bytes();
        let ping_round_trip_time = self.ping_round_trip_time.value().to_le_bytes();
        let request_tx_time = self.request_tx_time.value().to_le_bytes();
        let request_reply_time = self.request_reply_time.value().to_le_bytes();
        buf.put_slice(&ping_reply_time[0..5]);
        buf.put_slice(&ping_round_trip_time[0..5]);
        buf.put_slice(&request_tx_time[0..5]);
        buf.put_slice(&request_reply_time[0..5]);
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for RangingResponse {
    type Output = RangingResponse;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID, {});

        let mut ping_reply_time = eat_duration!(buf);
        let mut ping_round_trip_time = eat_duration!(buf);
        let mut request_tx_time = eat_instant!(buf);
        let mut request_reply_time = eat_duration!(buf);

        Ok(RangingResponse{ ping_reply_time, ping_round_trip_time, request_tx_time, request_reply_time })
    }
}

impl Serialize for DummyMessage {
    type Error = Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, Self::ID, Self::SIZE + self.length as usize, {});
        buf.put_nothing(self.length as usize);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        self.length as usize + 1
    }
}

use crate::tasks::canbus::ForwardEntry;
use vhrdcan::{FrameId, Frame, RawFrame};
use vhrdcan::id::{StandardId, ExtendedId};

impl Serialize for ForwardEntry {
    type Error = Error;

    // StandardId => |0000_0sss|s7:0|len|[data]
    // ExtendedId => |100e_eeee|e|e|e|len|[data]
    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        put_id!(buf, 0xcb, self.size_hint(), {});
        match self.frame.id() {
            FrameId::Standard(sid) => {
                buf.put_u16_be(sid.id());
            }
            FrameId::Extended(eid) => {
                buf.put_u32_be(eid.id() | (1 << 31));
            }
        }
        buf.put_u8(self.frame.len() as u8);
        buf.put_slice(self.frame.data());
        Ok(())
    }

    fn size_hint(&self) -> usize {
        let mut len = self.frame.len() + 2;
        match self.frame.id() {
            FrameId::Standard(_) => { len += 2; }
            FrameId::Extended(_) => { len += 4; }
        }
        len
    }
}
impl Deserialize for ForwardEntry {
    type Output = RawFrame;
    type Error = Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, 0xcb, {
            if buf.remaining() < 3 {
                return Err(Error::Eof);
            }
        });
        let maybe_sid = buf.get_u16_be();
        let id = if maybe_sid & (1 << 15) == 0 { // sid
            FrameId::Standard(unsafe { StandardId::new_unchecked(maybe_sid & 0x7_ff) })
        } else { // eid
            if buf.remaining() < 3 {
                return Err(Error::Eof);
            }
            let eid15_0 = buf.get_u16_be() as u32;
            FrameId::Extended(unsafe { ExtendedId::new_unchecked(((maybe_sid as u32 & 0x1F_FF) << 16) | eid15_0) })
        };
        let len = buf.get_u8();
        if buf.remaining() < len as usize {
            return Err(Error::Eof);
        }
        let mut data = [0u8; 8];
        data[0..len as usize].copy_from_slice(buf.slice_to(len as usize));
        Ok(RawFrame {
            id,
            data,
            len
        })
    }
}
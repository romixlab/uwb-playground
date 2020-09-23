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
use crate::radio::types::Pong;
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

macro_rules! eat_id {
    ($buf:ident, $id:expr) => {
        if $buf.remaining() >= 1 {
            if $buf.peek_u8() != $id {
                return Err(Error::WrongId);
            }
        } else {
            return Err(Error::Eof);
        }
        let _id = $buf.get_u8();
    };
}

impl Serialize for Pong {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for Pong {
    type Output = Pong;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID);
        Ok(Pong{})
    }
}

impl Serialize for Slot {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        if self.shift.0 >= core::u16::MAX as u32 || self.duration.0 >= core::u16::MAX as u32 {
            return Err(Error::WindowTooLong);
        }
        buf.put_u8(Self::ID);
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
        if buf.remaining() < self.len() + 1 { // 1 byte for ID
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(0xfd);
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
        eat_id!(buf, Self::ID);
        if buf.remaining() < Self::SIZE - 1 {
            return Err(Error::Eof);
        }
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

impl Serialize for RangingPing {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        let tx_time = self.ping_tx_time.value().to_le_bytes();
        buf.put_slice(&tx_time[0..5]);
        Ok(())
    }

    fn size_hint(&self) -> usize { Self::SIZE }
}

impl Deserialize for RangingPing {
    type Output = RangingPing;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        eat_id!(buf, Self::ID);
        let mut tx_time = [0u8; 8];
        tx_time[0..5].copy_from_slice(buf.slice_to(5));
        let tx_time = u64::from_le_bytes(tx_time);
        buf.advance(5);
        let tx_time = dw1000::time::Instant::new(tx_time).unwrap();
        Ok(RangingPing{ ping_tx_time: tx_time })
    }
}
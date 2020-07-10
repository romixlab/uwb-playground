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
    Window,
    WindowType,
    RadioConfig,
};
use super::Error;
use crate::units::MicroSeconds;

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

impl MessageSpec for Window {
    const ID: u8 = 0x70;
    const SIZE: usize = 6;
}

impl Serialize for Window {
    type Error = super::Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        if self.shift.0 >= core::u16::MAX as u32 || self.window.0 >= core::u16::MAX as u32 {
            return Err(Error::WindowTooLong);
        }
        buf.put_u8(Self::ID);
        let shift = self.shift.0 as u16;
        let window = self.window.0 as u16;
        buf.put_u16(shift);
        buf.put_u16(window);
        let channel = self.radio_config.channel as u8;
        let bitrate = self.radio_config.bitrate as u8;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if self.radio_config.prf == Mhz16 { 0u8 } else { 1u8 };
        let window_type = self.window_type as u8;
        buf.put_u8((window_type << 7) | (prf << 6) | (bitrate << 4) | channel);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        Self::SIZE
    }
}

impl Deserialize for Window {
    type Output = Window;
    type Error = super::Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        if buf.remaining() >= 1 {
            if buf.peek_u8() != Self::ID {
                return Err(Error::WrongId);
            }
        } else {
            return Err(Error::Eof);
        }
        if buf.remaining() < Self::SIZE {
            return Err(Error::Eof);
        }

        let _id = buf.get_u8();
        let shift = buf.get_u16();
        let window = buf.get_u16();
        let config = buf.get_u8();
        let channel = channel_from_u8(config & 0b0000_1111).ok_or(Error::WrongChannel)?;
        let bitrate = bitrate_from_u8((config & 0b0011_0000) >> 4).ok_or(Error::WrongBitrate)?;
        use dw1000::configs::PulseRepetitionFrequency::*;
        let prf = if config & 0b0100_0000 == 0 { Mhz16 } else { Mhz64 };
        use WindowType::*;
        let window_type = if config & 0b1000_0000 == 0 { Uplink } else { Downlink };
        Ok(Window {
            shift: MicroSeconds(shift as u32),
            window: MicroSeconds(window as u32),
            window_type,
            radio_config: RadioConfig {
                channel,
                bitrate,
                prf
            }
        })
    }
}
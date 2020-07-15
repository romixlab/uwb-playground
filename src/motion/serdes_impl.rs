use crate::radio::serdes::{
    Serialize,
    Deserialize,
    Buf,
    BufMut,
    MessageSpec,
};
use crate::radio::Error;
use crate::units::{MicroSeconds, Watts};
use super::Tachometer;
use crate::motion::{Rpm, TelemetryItem};

impl MessageSpec for Tachometer {
    const ID: u8 = 0x71;
    const SIZE: usize = 5;
}

impl MessageSpec for Rpm {
    const ID: u8 = 0x72;
    const SIZE: usize = 5;
}

impl MessageSpec for Watts {
    const ID: u8 = 0x73;
    const SIZE: usize = 5;
}

impl MessageSpec for TelemetryItem {
    const ID: u8 = 0x74;
    const SIZE: usize = 0;
}

impl Serialize for Tachometer {
    type Error = Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        buf.put_i32(self.0);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        Self::SIZE
    }
}

impl Deserialize for Tachometer {
    type Output = Tachometer;
    type Error = Error;

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
        let tacho = buf.get_i32();
        Ok(Tachometer(tacho))
    }
}

impl Serialize for Rpm {
    type Error = Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        buf.put_i32(self.0);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        Self::SIZE
    }
}

impl Deserialize for Rpm {
    type Output = Rpm;
    type Error = Error;

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
        let rpm = buf.get_i32();
        Ok(Rpm(rpm))
    }
}

impl Serialize for TelemetryItem {
    type Error = Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < self.size_hint() {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        match self {
            TelemetryItem::Tachometer(tacho) => { tacho.ser(buf)?; },
            TelemetryItem::Rpm(rpm) => { rpm.ser(buf)?; },
            TelemetryItem::Power(power) => { power.ser(buf)?; },
        }
        Ok(())
    }

    fn size_hint(&self) -> usize {
        1 + match self {
            TelemetryItem::Tachometer(_) => { Tachometer::SIZE },
            TelemetryItem::Rpm(_) => { Rpm::SIZE },
            TelemetryItem::Power(_) => { Watts::SIZE },
        }
    }
}

macro_rules! ok_or_continue {
    ($e:expr, $v:expr) => {
        match $e {
            Ok(r) => { return Ok($v(r)); },
            Err(_) => {}
        }
    };
}

impl Deserialize for TelemetryItem {
    type Output = TelemetryItem;
    type Error = Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error> {
        if buf.remaining() >= 1 {
            if buf.peek_u8() != Self::ID {
                return Err(Error::WrongId);
            }
        } else {
            return Err(Error::Eof);
        }
        let _id = buf.get_u8();
        ok_or_continue!(Tachometer::des(buf), TelemetryItem::Tachometer);
        ok_or_continue!(Rpm::des(buf), TelemetryItem::Rpm);
        ok_or_continue!(Watts::des(buf), TelemetryItem::Power);
        Err(Error::WrongDiscriminant)
    }
}

impl Serialize for Watts {
    type Error = Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error> {
        if buf.remaining() < Self::SIZE {
            return Err(Error::NotEnoughSpace);
        }
        buf.put_u8(Self::ID);
        buf.put_f32(self.0);
        Ok(())
    }

    fn size_hint(&self) -> usize {
        Self::SIZE
    }
}

impl Deserialize for Watts {
    type Output = Watts;
    type Error = Error;

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
        let watts = buf.get_f32();
        Ok(Watts(watts))
    }
}
use crate::radio::serdes::{
    Serialize,
    Deserialize,
    Buf,
    BufMut,
    MessageSpec,
};
use crate::radio::Error;
use crate::units::MicroSeconds;
use super::Tachometer;

impl MessageSpec for Tachometer {
    const ID: u8 = 0x71;
    const SIZE: usize = 5;
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
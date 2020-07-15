pub trait MessageSpec {
    const ID: u8;
    const SIZE: usize;
}

pub trait Serialize {
    type Error;

    fn ser(&self, buf: &mut BufMut) -> Result<(), Self::Error>;
    fn size_hint(&self) -> usize;
}

pub trait Deserialize {
    type Output;
    type Error;

    fn des(buf: &mut Buf) -> Result<Self::Output, Self::Error>;
}

pub struct Buf<'a> {
    buf: &'a [u8],
    idx: usize
}

impl<'a> Buf<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Buf {
            buf, idx: 0
        }
    }

    pub fn remaining(&self) -> usize {
        self.buf.len() - self.idx
    }

    pub fn written(&self) -> usize {
        self.idx
    }

    pub fn slice_to(&self, len: usize) -> &'a [u8] {
        unsafe { core::slice::from_raw_parts(self.buf.get_unchecked(self.idx), len) }
    }

    pub fn advance(&mut self, cnt: usize) {
        self.idx += cnt;
    }

    pub fn get_u8(&mut self) -> u8 {
        let val = unsafe { *self.buf.get_unchecked(self.idx) };
        self.idx += 1;
        val
    }

    pub fn peek_u8(&self) -> u8 {
        unsafe { *self.buf.get_unchecked(self.idx) }
    }

    pub fn get_u16(&mut self) -> u16 {
        let mut le = unsafe {
            [
                *self.buf.get_unchecked(self.idx),
                *self.buf.get_unchecked(self.idx + 1)
            ]
        };
        self.idx += 2;
        u16::from_le_bytes(le)
    }

    pub fn get_u32(&mut self) -> u32 {
        let mut le = unsafe {
            [
                *self.buf.get_unchecked(self.idx),
                *self.buf.get_unchecked(self.idx + 1),
                *self.buf.get_unchecked(self.idx + 2),
                *self.buf.get_unchecked(self.idx + 3)
            ]
        };
        self.idx += 4;
        u32::from_le_bytes(le)
    }

    pub fn get_i32(&mut self) -> i32 {
        let mut le = unsafe {
            [
                *self.buf.get_unchecked(self.idx),
                *self.buf.get_unchecked(self.idx + 1),
                *self.buf.get_unchecked(self.idx + 2),
                *self.buf.get_unchecked(self.idx + 3)
            ]
        };
        self.idx += 4;
        i32::from_le_bytes(le)
    }

    pub fn get_f32(&mut self) -> f32 {
        let bits = self.get_u32();
        f32::from_bits(bits)
    }
}

pub struct BufMut<'a> {
    buf: &'a mut [u8],
    idx: usize
}

impl<'a> BufMut<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        BufMut {
            buf, idx: 0
        }
    }

    pub fn remaining(&self) -> usize {
        self.buf.len() - self.idx
    }

    pub fn written(&self) -> usize {
        self.idx
    }

    pub fn put_u8(&mut self, val: u8) {
        unsafe {
            *self.buf.get_unchecked_mut(self.idx) = val;
        }
        self.idx += 1;
    }

    pub fn put_u16(&mut self, val: u16) {
        let le = val.to_le_bytes();
        unsafe {
            *self.buf.get_unchecked_mut(self.idx) = le[0];
            *self.buf.get_unchecked_mut(self.idx + 1) = le[1];
        }
        self.idx += 2;
    }

    pub fn put_u32(&mut self, val: u32) {
        let le = val.to_le_bytes();
        unsafe {
            *self.buf.get_unchecked_mut(self.idx) = le[0];
            *self.buf.get_unchecked_mut(self.idx + 1) = le[1];
            *self.buf.get_unchecked_mut(self.idx + 2) = le[2];
            *self.buf.get_unchecked_mut(self.idx + 3) = le[3];
        }
        self.idx += 4;
    }

    pub fn put_i32(&mut self, val: i32) {
        let le = val.to_le_bytes();
        unsafe {
            *self.buf.get_unchecked_mut(self.idx) = le[0];
            *self.buf.get_unchecked_mut(self.idx + 1) = le[1];
            *self.buf.get_unchecked_mut(self.idx + 2) = le[2];
            *self.buf.get_unchecked_mut(self.idx + 3) = le[3];
        }
        self.idx += 4;
    }

    pub fn put_f32(&mut self, val: f32) {
        self.put_u32(val.to_bits());
    }

    pub fn put_slice(&mut self, s: &[u8]) {
        unsafe {
            core::ptr::copy(
                s.as_ptr(),
                self.buf.as_mut_ptr().offset(self.idx as isize),
                s.len()
            );
        }
        self.idx += s.len();
    }
}

// use serde::{ser::{SerializeStruct, Serializer}};
// impl serde::ser::Serialize for super::message::Window {
//     fn serialize<S>(&self, s: S) -> Result<<S as Serializer>::Ok, <S as Serializer>::Error> where
//         S: Serializer
//     {
//         let mut s = s.serialize_struct("", 4)?;
//         s.serialize_field("", &Self::ID)?;
//         let shift = self.shift.0 as u16;
//         let window = self.window.0 as u16;
//         s.serialize_field("", &shift)?;
//         s.serialize_field("", &window)?;
//         let channel = self.radio_config.channel as u8;
//         let bitrate = self.radio_config.bitrate as u8;
//         use dw1000::configs::PulseRepetitionFrequency::*;
//         let prf = if self.radio_config.prf == Mhz16 { 0u8 } else { 1u8 };
//         let window_type = self.window_type as u8;
//         let packed = (window_type << 7) | (prf << 6) | (bitrate << 4) | channel;
//         s.serialize_field("", &packed)?;
//         s.end()
//     }
// }
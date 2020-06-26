use generic_array::{GenericArray, ArrayLength};
use core::convert::TryInto;
use crc16;
//use rtt_target::{rprintln};
use core::ops::Range;

pub struct CrcFramerDe<N: ArrayLength<u8>> {
    buffer: GenericArray<u8, N>,
    read_idx: usize,
    write_idx: usize,
    bytes_left: usize
}

impl<N: generic_array::ArrayLength<u8>> CrcFramerDe<N> {
    pub fn new() -> Self {
        CrcFramerDe {
            buffer: GenericArray::default(),
            read_idx: 0, write_idx: 0, bytes_left: 0
        }
    }

    pub fn eat_byte<F>(&mut self, byte: u8, mut f: F)
        where F: FnMut(&[u8])
    {
        //rprintln!("\n\neat: {:02x}", byte);
        let mut bytes_pending = self.write_idx - self.read_idx;
        // Incoming frame is larger than the buffer
        if bytes_pending >= N::to_usize() {
            self.write_idx = 1;
            self.read_idx = 0;
            self.bytes_left = 0;
            self.buffer[0] = byte;
            return;
        }
        // Move part of the frame in the tail to the head (only when frame is wrapping around after junk bytes)
        if self.write_idx >= N::to_usize() {
            unsafe {
                core::ptr::copy(
                    self.buffer.as_ptr().offset(self.read_idx as isize),
                    self.buffer.as_mut_ptr(),
                    bytes_pending);
            }
            self.read_idx = 0;
            self.write_idx = bytes_pending;
        }
        // Save incoming byte
        self.buffer[self.write_idx] = byte;
        self.write_idx += 1;
        bytes_pending += 1;
        // Valid frame boundary is potentially found, return till enough bytes arrive to make progress
        if self.bytes_left > 1 {
            self.bytes_left -= 1;
            return;
        }
        // Search for frame boundary when unsynchronised or just check crc and emit valid frames
        let mut lookahead_len = bytes_pending;
        for _ in self.read_idx..self.read_idx + bytes_pending {
            //rprintln!("___");
            let result = self.decode_frame(lookahead_len);
            match result {
                DecodeResult::NeedMoreBytes => { return; }, // probably wrong if junk was recognized as frame start and followed by a good frame
                DecodeResult::InvalidData => {
                    self.read_idx += 1;
                    lookahead_len -= 1;
                },
                DecodeResult::Consumed(count, range) => {
                    f(&self.buffer[range]);
                    lookahead_len -= count;
                    self.read_idx += count;
                },
            }
        }

        if bytes_pending == 0 {
            self.read_idx = 0;
            self.write_idx = 0;
        }
    }

    fn decode_frame(&mut self, data_len: usize) -> DecodeResult
    {
        use DecodeResult::*;
        // Need at least 1 byte
        if data_len == 0 {
            self.bytes_left = 1;
            //rprintln!("T2");
            return NeedMoreBytes;
        } else {
            self.bytes_left = 0;
        }
        // Check start byte
        let b0 = self.buffer[self.read_idx];
        let is_len_8b = b0 == 2;
        let is_len_16b = b0 == 3;
        let is_len_24b = b0 == 4;
        if !is_len_8b && !is_len_16b && !is_len_24b {
            //rprintln!("T3");
            return InvalidData;
        }
        // Ignore too big frames right away
        if is_len_24b {
            //rprintln!("T4");
            return InvalidData;
        }
        // Not enough bytes to determine length
        if data_len < b0 as usize {
            self.bytes_left = b0 as usize - data_len;
            //rprintln!("T5");
            return NeedMoreBytes;
        }
        let frame_len = if is_len_8b {
            let len = self.buffer[self.read_idx + 1];
            if len == 0 {
                //rprintln!("T6");
                return InvalidData;
            }
            len as usize
        } else { // 16b
            let beu16: [u8; 2] = self.buffer[self.read_idx + 1 ..= self.read_idx + 2].try_into().unwrap();
            let len = u16::from_be_bytes(beu16);
            if len < 255 {
                //rprintln!("T7");
                return InvalidData;
            }
            len as usize
        };
        //rprintln!("frame_len: {}", frame_len);
        // Ignore too big frames
        if frame_len > N::to_usize() {
            //rprintln!("T8");
            return InvalidData;
        }
        // Rest of the frame
        if data_len < frame_len + b0 as usize + 3 {
            self.bytes_left = frame_len + b0 as usize + 3 - data_len;
            //rprintln!("T9");
            return NeedMoreBytes;
        }
        // Invalid stop byte
        if self.buffer[self.read_idx + b0 as usize + frame_len + 2] != 3 {
            //rprintln!("T10");
            return InvalidData;
        }
        // Check CRC
        let received_crc: [u8; 2] = self.buffer[
                self.read_idx + frame_len + b0 as usize ..=
                self.read_idx + frame_len + b0 as usize + 1
        ].try_into().unwrap();
        let received_crc = u16::from_be_bytes(received_crc);
        let crc = crc16::State::<crc16::XMODEM>::calculate(
            &self.buffer[self.read_idx + b0 as usize .. self.read_idx + b0 as usize + frame_len]
        );
        if crc == received_crc {
            //rprintln!("vesc_valid");
            Consumed(
                frame_len + b0 as usize + 3,
                Range{
                    start: self.read_idx + b0 as usize,
                    end: self.read_idx + b0 as usize + frame_len
                }
            )
        } else {
            //rprintln!("crc r:{:04x} c:{:04x}", received_crc, crc);
            InvalidData
        }
    }
}

enum DecodeResult {
    NeedMoreBytes,
    InvalidData,
    Consumed(usize, Range<usize>)
}

pub struct CrcFramerSer {

}

pub enum CrcFramerError {
    InvalidLength,
    NotEnoughSpace,
}

impl CrcFramerSer {
    pub fn commit_frame<N, I>(
        frame: &[u8],
        producer: &mut bbqueue::Producer<'static, N>,
        consumer_irq: I
    ) -> core::result::Result<(), CrcFramerError>
        where
            N: generic_array::ArrayLength<u8>,
            I: cortex_m::interrupt::Nr
    {
        let (bytes_required, first_byte) = if frame.len() <= 255 {
            (2 + frame.len() + 3, 2u8)
        } else if frame.len() >= 256 && frame.len() <= N::to_usize() {
            (3 + frame.len() + 3, 3u8)
        } else {
            return Err(CrcFramerError::InvalidLength);
        };
        let mut wgr = producer.grant_exact(bytes_required).map_err(|_| CrcFramerError::NotEnoughSpace)?;
        wgr[0] = first_byte;
        if frame.len() <= 255 {
            wgr[1] = frame.len() as u8;
        } else {
            let lenbe: [u8; 2] = (frame.len() as u16).to_be_bytes();
            wgr[1] = lenbe[0];
            wgr[2] = lenbe[1];
        }
        let data_start_idx = first_byte as usize;
        wgr[data_start_idx .. data_start_idx + frame.len()].copy_from_slice(frame);
        let crc: u16 = crc16::State::<crc16::XMODEM>::calculate(frame);
        let crc_start_idx = data_start_idx + frame.len();
        wgr[crc_start_idx ..= crc_start_idx + 1].copy_from_slice(&crc.to_be_bytes());
        wgr[crc_start_idx + 2] = 3;
        wgr.commit(bytes_required);
        rtic::pend(consumer_irq);
        Ok(())
    }
}
//! Module related to drivers for driving a 7-segment display with the HT16K33 chip. The most
//! common setup that uses the HT16K33 is Adafruit's 0.56" 7-segment display backpack which uses
//! the HT16K33 to provide I2C communication from microcontrollers to the display.
//!
//! The module contains two types of variants, and syncrhonous driver and an asynchronous driver.

use alloc::vec;
use core::usize;

use embedded_hal::i2c::I2c as SyncI2c;
use embedded_hal_async::i2c::I2c as AsyncI2c;

mod digit_segment_encoding {
    pub const DOT: u16 = 0b10000000;
    pub const ZERO: u16 = 0b00111111;
    pub const ONE: u16 = 0b00000110;
    pub const TWO: u16 = 0b01011011;
    pub const THREE: u16 = 0b01001111;
    pub const FOUR: u16 = 0b01100110;
    pub const FIVE: u16 = 0b01101101;
    pub const SIX: u16 = 0b01111101;
    pub const SEVEN: u16 = 0b00000111;
    pub const EIGHT: u16 = 0b01111111;
    pub const NINE: u16 = 0b01101111;

    pub fn digit_to_mask(digit: u8) -> u16 {
        match digit {
            0 => ZERO,
            1 => ONE,
            2 => TWO,
            3 => THREE,
            4 => FOUR,
            5 => FIVE,
            6 => SIX,
            7 => SEVEN,
            8 => EIGHT,
            9 => NINE,
            _ => 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum H16K33Blinkrate {
    BlinkOff = 0,
    Blink2Hz,
    Blink1Hz,
    BlinkHalfHz,
}

const HT16K33_BASE_CMD: u8 = 0x70;
const HT16K33_DISPLAY_ON: u8 = 0x01;

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
enum HT16K33Commands {
    Begin = 0x21,
    SetSegments = 0x00,
    SetBlink = 0x80,
    SetBrightness = 0xE0,
}

/// A syncrhonous driver for the HT16K33 driving a 7-segment display. Can be used with any I2C
/// interface that implements the `embedded_hal::i2c::I2c` trait.
pub struct SyncI2C7SegDisplay<const DISPLAY_SIZE: usize, T: SyncI2c> {
    address_offset: u8,
    tx: T,
}

impl<const DISPLAY_SIZE: usize, T: SyncI2c> SyncI2C7SegDisplay<DISPLAY_SIZE, T> {
    pub fn new(address_offset: u8, tx: T) -> Self {
        Self { address_offset, tx }
    }

    pub fn initialize(&mut self) -> Result<(), T::Error> {
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            &[HT16K33Commands::Begin as u8],
        )?;
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            &[HT16K33Commands::SetBlink as u8 | 1],
        )?;
        self.clear_display()?;
        Ok(())
    }

    pub fn enable(&mut self, turn_on: bool) -> Result<(), T::Error> {
        let mut buf = [0; 1];
        buf[0] = if turn_on {
            HT16K33Commands::SetBlink as u8 | HT16K33_DISPLAY_ON
        } else {
            HT16K33Commands::SetBlink as u8
        };

        self.tx.write(HT16K33_BASE_CMD + self.address_offset, &buf)
    }

    pub fn clear_display(&mut self) -> Result<(), T::Error> {
        let mut empty_segment_command = [0; 17];
        empty_segment_command[0] = HT16K33Commands::SetSegments as u8;
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            &empty_segment_command,
        )
    }

    pub fn set_brightness(&mut self, mut brightness: u8) -> Result<(), T::Error> {
        const MAX_BRIGHTNESS: u8 = 0xf;
        if brightness > MAX_BRIGHTNESS {
            brightness = MAX_BRIGHTNESS;
        }
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            &[HT16K33Commands::SetBrightness as u8 | brightness],
        )
    }

    pub fn set_blinkrate(&mut self, blinkrate: H16K33Blinkrate) -> Result<(), T::Error> {
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            &[HT16K33Commands::SetBlink as u8 | HT16K33_DISPLAY_ON | ((blinkrate as u8) << 1)],
        )
    }

    pub fn write_f64(&mut self, float: f64, precision: u32) -> Result<(), T::Error>
    where
        [(); DISPLAY_SIZE * 2 + 1]: Sized,
    {
        let buf: [u8; DISPLAY_SIZE * 2 + 1] = float_to_segment_buffer(float, precision);

        // FIXME: Figure out a better way of distinguishing dedicated colon(s) in display (possibly
        // on construction)
        let (lhs, rhs) = buf.split_at(DISPLAY_SIZE + 1);
        self.tx.write(
            HT16K33_BASE_CMD + self.address_offset,
            vec![lhs, &[0x00, 0x00], rhs].concat().as_slice(),
        )
    }
}

/// An asyncrhonous driver for the HT16K33 driving a 7-segment display. Can be used with any I2C
/// interface that implements the `embedded_hal_async::i2c::I2c` trait.
pub struct AsyncI2C7SegDisplay<const DISPLAY_SIZE: usize, T: AsyncI2c> {
    address_offset: u8,
    tx: T,
}

impl<const DISPLAY_SIZE: usize, T: AsyncI2c> AsyncI2C7SegDisplay<DISPLAY_SIZE, T> {
    pub fn new(address_offset: u8, tx: T) -> Self {
        Self { address_offset, tx }
    }

    pub async fn initialize(&mut self) -> Result<(), T::Error> {
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[HT16K33Commands::Begin as u8],
            )
            .await?;
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[HT16K33Commands::SetBlink as u8 | 1],
            )
            .await?;
        self.clear_display().await?;
        Ok(())
    }

    pub async fn enable(&mut self, turn_on: bool) -> Result<(), T::Error> {
        let mut buf = [0; 1];
        buf[0] = if turn_on {
            HT16K33Commands::SetBlink as u8 | HT16K33_DISPLAY_ON
        } else {
            HT16K33Commands::SetBlink as u8
        };

        self.tx
            .write(HT16K33_BASE_CMD + self.address_offset, &buf)
            .await
    }

    pub async fn clear_display(&mut self) -> Result<(), T::Error> {
        let mut empty_segment_command = [0; 17];
        empty_segment_command[0] = HT16K33Commands::SetSegments as u8;
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &empty_segment_command,
            )
            .await
    }

    pub async fn set_brightness(&mut self, mut brightness: u8) -> Result<(), T::Error> {
        const MAX_BRIGHTNESS: u8 = 0xf;
        if brightness > MAX_BRIGHTNESS {
            brightness = MAX_BRIGHTNESS;
        }
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[HT16K33Commands::SetBrightness as u8 | brightness],
            )
            .await
    }

    pub async fn set_blinkrate(&mut self, blinkrate: H16K33Blinkrate) -> Result<(), T::Error> {
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[HT16K33Commands::SetBlink as u8 | HT16K33_DISPLAY_ON | ((blinkrate as u8) << 1)],
            )
            .await
    }

    pub async fn write_f64(&mut self, float: f64, precision: u32) -> Result<(), T::Error>
    where
        [(); DISPLAY_SIZE * 2 + 1]: Sized,
    {
        let buf: [u8; DISPLAY_SIZE * 2 + 1] = float_to_segment_buffer(float, precision);

        // FIXME: Figure out a better way of distinguishing dedicated colon(s) in display (possibly
        // on construction)
        let (lhs, rhs) = buf.split_at(DISPLAY_SIZE + 1);
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                vec![lhs, &[0x00, 0x00], rhs].concat().as_slice(),
            )
            .await
    }
}

fn float_to_segment_buffer<const N: usize>(float: f64, precision: u32) -> [u8; N] {
    let is_neg = float.is_sign_negative();
    let mut lhs = float as u64;
    let mut rhs = ((float - lhs as f64) * (10_u32.pow(precision) as f64)) as u64;
    // FIXME: The above u32 to f64 conversion is subject to floating point imprecision

    let mut buf: [u8; N] = [0; N];

    for (r_idx, chunk) in buf.rchunks_exact_mut(2).enumerate() {
        if rhs == 0 && r_idx < precision as usize {
            let le = digit_segment_encoding::ZERO.to_le_bytes();
            for (chunk_byte, byte) in chunk.iter_mut().zip(le) {
                *chunk_byte = byte;
            }
        } else {
            if rhs == 0 && lhs == 0 {
                if r_idx == precision as usize {
                    let mut encoding = digit_segment_encoding::ZERO;
                    encoding |= digit_segment_encoding::DOT;
                    let le = encoding.to_le_bytes();
                    for (chunk_byte, byte) in chunk.iter_mut().zip(le) {
                        *chunk_byte = byte;
                    }
                }
                break;
            }
            let n = if rhs > 0 { &mut rhs } else { &mut lhs };
            let digit = (*n % 10) as u8;
            let mut encoding = digit_segment_encoding::digit_to_mask(digit);
            if r_idx == precision as usize {
                encoding |= digit_segment_encoding::DOT;
            }
            let le = encoding.to_le_bytes();
            for (chunk_byte, byte) in chunk.iter_mut().zip(le) {
                *chunk_byte = byte;
            }
            *n /= 10;
        }
    }
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_float_to_segment_buffer() {
        let f = 27.0;
        let precision = 2;
        const DISPLAY_SIZE: usize = 4;
        const BUF_SIZE: usize = DISPLAY_SIZE * 2 + 1;
        let buf = float_to_segment_buffer::<BUF_SIZE>(f, precision);
        let expected_buf = [
            HT16K33Commands::SetSegments as u8,
            (digit_segment_encoding::TWO & 0xff) as u8,
            (digit_segment_encoding::TWO >> 8) as u8,
            (digit_segment_encoding::SEVEN & 0xff) as u8
                | (digit_segment_encoding::DOT & 0xff) as u8,
            (digit_segment_encoding::SEVEN >> 8) as u8 | (digit_segment_encoding::DOT >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8,
        ];

        assert_eq!(buf, expected_buf);
    }

    #[test]
    fn leading_zero_float_to_segment_buffer() {
        let f = 0.25;
        let precision = 3;
        const DISPLAY_SIZE: usize = 4;
        const BUF_SIZE: usize = DISPLAY_SIZE * 2 + 1;
        let buf = float_to_segment_buffer::<BUF_SIZE>(f, precision);
        let expected_buf = [
            HT16K33Commands::SetSegments as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8
                | (digit_segment_encoding::DOT & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8 | (digit_segment_encoding::DOT >> 8) as u8,
            (digit_segment_encoding::TWO & 0xff) as u8,
            (digit_segment_encoding::TWO >> 8) as u8,
            (digit_segment_encoding::FIVE & 0xff) as u8,
            (digit_segment_encoding::FIVE >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8,
        ];

        assert_eq!(buf, expected_buf);
    }

    #[test]
    fn right_aligned_float_to_segment_buffer() {
        let f = 60.5;
        let precision = 1;
        const DISPLAY_SIZE: usize = 4;
        const BUF_SIZE: usize = DISPLAY_SIZE * 2 + 1;
        let buf = float_to_segment_buffer::<BUF_SIZE>(f, precision);
        let expected_buf = [
            HT16K33Commands::SetSegments as u8,
            0x0,
            0x0,
            (digit_segment_encoding::SIX & 0xff) as u8,
            (digit_segment_encoding::SIX >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8
                | (digit_segment_encoding::DOT & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8 | (digit_segment_encoding::DOT >> 8) as u8,
            (digit_segment_encoding::FIVE & 0xff) as u8,
            (digit_segment_encoding::FIVE >> 8) as u8,
        ];

        assert_eq!(buf, expected_buf);
    }

    #[test]
    fn precise_float_to_segment_buffer() {
        let f = 10.2;
        let precision = 2;
        const DISPLAY_SIZE: usize = 4;
        const BUF_SIZE: usize = DISPLAY_SIZE * 2 + 1;
        let buf = float_to_segment_buffer::<BUF_SIZE>(f, precision);
        let expected_buf = [
            HT16K33Commands::SetSegments as u8,
            (digit_segment_encoding::ONE & 0xff) as u8,
            (digit_segment_encoding::ONE >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8
                | (digit_segment_encoding::DOT & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8 | (digit_segment_encoding::DOT >> 8) as u8,
            (digit_segment_encoding::TWO & 0xff) as u8,
            (digit_segment_encoding::TWO >> 8) as u8,
            (digit_segment_encoding::ZERO & 0xff) as u8,
            (digit_segment_encoding::ZERO >> 8) as u8,
        ];

        assert_eq!(buf, expected_buf);
    }
}

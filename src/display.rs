use core::usize;

use esp_hal::i2c::Error;

use embedded_hal_async::i2c::I2c;
use esp_hal::i2c::I2C;
use esp_hal::{i2c, peripherals::I2C0};

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

pub struct I2C7SegDisplay<const DISPLAY_SIZE: usize> {
    address_offset: u8,
    tx: i2c::I2C<'static, I2C0>,
}

impl<const DISPLAY_SIZE: usize> I2C7SegDisplay<DISPLAY_SIZE> {
    pub fn new(address_offset: u8, tx: I2C<'static, I2C0>) -> Self {
        Self { address_offset, tx }
    }

    pub async fn initialize(&mut self) -> Result<(), Error> {
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

        let mut empty_segment_command = [0; 17];
        empty_segment_command[0] = HT16K33Commands::SetSegments as u8;
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &empty_segment_command,
            )
            .await?;
        Ok(())
    }

    pub async fn enable(&mut self, turn_on: bool) -> Result<(), Error> {
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

    pub async fn set_brightness(&mut self, mut brightness: u8) -> Result<(), Error> {
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

    pub async fn set_blinkrate(&mut self, blinkrate: H16K33Blinkrate) -> Result<(), Error> {
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[HT16K33Commands::SetBlink as u8 | HT16K33_DISPLAY_ON | ((blinkrate as u8) << 1)],
            )
            .await
    }

    pub async fn write_f64(&mut self, float: f64, precision: u32) -> Result<(), Error>
    where
        [(); DISPLAY_SIZE * 2 + 1]: Sized,
    {
        let is_neg = float.is_sign_negative();
        let mut lhs = float as u64;
        let mut rhs = ((float - lhs as f64) * (10_u32.pow(precision) as f64)) as u64;
        // FIXME: The above u32 to f64 conversion is subject to floating point imprecision

        let mut buf: [u8; DISPLAY_SIZE * 2 + 1] = [0; DISPLAY_SIZE * 2 + 1];

        for (r_idx, chunk) in buf.rchunks_exact_mut(2).enumerate() {
            if rhs == 0 && r_idx < precision as usize {
                let le = to_little_endian(digit_segment_encoding::ZERO);
                for (chunk_byte, byte) in chunk.iter_mut().zip(le) {
                    *chunk_byte = byte;
                }
            } else {
                if rhs == 0 && lhs == 0 {
                    if r_idx == precision as usize {
                        let mut encoding = digit_segment_encoding::ZERO;
                        encoding |= digit_segment_encoding::DOT;
                        let le = to_little_endian(encoding);
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
                let le = to_little_endian(encoding);
                for (chunk_byte, byte) in chunk.iter_mut().zip(le) {
                    *chunk_byte = byte;
                }
                *n /= 10;
            }
        }

        // FIXME: Figure out a better way of distinguishing dedicated colon(s) in display (possibly
        // on construction)
        let (lhs, rhs) = buf.split_at(DISPLAY_SIZE + 1);
        self.tx
            .write(
                HT16K33_BASE_CMD + self.address_offset,
                &[lhs, &[0x00, 0x00], rhs].concat(),
            )
            .await
    }
}

fn to_little_endian(mut n: u16) -> [u8; 2] {
    let mut buf = [0; 2];
    buf.iter_mut().for_each(|byte| {
        *byte = (n & 0xff) as u8;
        n >>= 8;
    });
    buf
}

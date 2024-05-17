use esp_hal::i2c::Error;

use embedded_hal_async::i2c::I2c;
use esp_hal::i2c::I2C;
use esp_hal::{i2c, peripherals::I2C0};

mod digit_segment_encoding {
    pub const DOT: u8 = 0b10000000;
    pub const ZERO: u8 = 0b00111111;
    pub const ONE: u8 = 0b00000110; // 1
    pub const TWO: u8 = 0b01011011; // 2
    pub const THREE: u8 = 0b01001111; // 3
    pub const FOUR: u8 = 0b01100110; // 4
    pub const FIVE: u8 = 0b01101101; // 5
    pub const SIX: u8 = 0b01111101; // 6
    pub const SEVEN: u8 = 0b00000111; // 7
    pub const EIGHT: u8 = 0b01111111; // 8
    pub const NINE: u8 = 0b01101111; // 9

    pub fn digit_to_mask(digit: u8) -> u8 {
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
const HT16K33_BLINK_CMD: u8 = 0x80;
const HT16K33_BRIGHTNESS_CMD: u8 = 0xE0;

pub struct I2C7SegDsiplay<const DISPLAY_SIZE: usize> {
    address_offset: u8,
    tx: i2c::I2C<'static, I2C0>,
}

impl<const DISPLAY_SIZE: usize> I2C7SegDsiplay<DISPLAY_SIZE> {
    pub fn new(address_offset: u8, tx: I2C<'static, I2C0>) -> Self {
        Self { address_offset, tx }
    }

    pub async fn set_brightness(&mut self, mut brightness: u8) -> Result<(), Error> {
        const MAX_BRIGHTNESS: u8 = 15;
        if brightness > MAX_BRIGHTNESS {
            brightness = MAX_BRIGHTNESS;
        }
        self.tx
            .write(HT16K33_BRIGHTNESS_CMD + self.address_offset, &[brightness])
            .await
    }

    pub async fn set_blinkrate(&mut self, blinkrate: H16K33Blinkrate) -> Result<(), Error> {
        self.tx.write(HT16K33_BLINK_CMD, &[blinkrate as u8]).await
    }

    pub async fn write_f64<const PRECISION: u32>(&mut self, float: f64) -> Result<(), Error> {
        let _ = assert!(DISPLAY_SIZE > (PRECISION as usize));

        let mut lhs = float as i64;
        let mut rhs = ((float - lhs as f64) * (10_u32.pow(PRECISION) as f64)) as u64;
        // FIXME: The above u33 to f64 conversion is subject to floating point imprecision

        let mut reserve_digits = DISPLAY_SIZE;
        let mut buf: [u8; DISPLAY_SIZE] = [0; DISPLAY_SIZE];

        while rhs > 0 {
            let digit = (rhs % 10) as u8;
            buf[reserve_digits - 1] = digit_segment_encoding::digit_to_mask(digit);
            rhs /= 10;
            reserve_digits -= 1;
        }
        if reserve_digits == 1 {
            buf[reserve_digits] = digit_segment_encoding::ZERO;
        }
        buf[reserve_digits] |= digit_segment_encoding::DOT;

        while lhs > 0 {
            let digit = (lhs % 10) as u8;
            buf[reserve_digits - 1] = digit_segment_encoding::digit_to_mask(digit);
            lhs /= 10;
            reserve_digits -= 1;
        }

        self.tx
            .write(HT16K33_BASE_CMD + self.address_offset, &buf)
            .await
    }
}

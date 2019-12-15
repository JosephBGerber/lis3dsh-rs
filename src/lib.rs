#![no_std]
#![allow(non_camel_case_types)]

use embedded_hal as hal;
use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

pub use accelerometer::{Accelerometer, F32x3, I16x3};

pub mod register;
pub use register::Register;

fn cast(value: i16) -> u8 {
    if value.abs() > i16::from(core::u8::MAX) {
        if value.is_positive() {
            return core::u8::MAX;
        } else {
            return core::u8::MIN;
        }
    }

    f32::from(value) as i8 as u8
}

/// Enum representing possible scales
#[derive(Copy, Clone, Debug)]
pub enum Scale {
    G2 = 0b000,
    G4 = 0b001,
    G6 = 0b010,
    G8 = 0b011,
    G16 = 0b100,
}

impl Scale {
    fn as_f32(self) -> f32 {
        match self {
            Scale::G2 => 2.0,
            Scale::G4 => 4.0,
            Scale::G6 => 6.0,
            Scale::G8 => 8.0,
            Scale::G16 => 16.0,
        }
    }
}

/// Enum representing possible sample rates
#[derive(Copy, Clone, Debug)]
pub enum SampleRate {
    ORD1 = 0b0000,
    ORD2 = 0b0001,
    ORD3 = 0b0010,
    ORD4 = 0b0011,
    ORD5 = 0b0100,
    ORD6 = 0b0101,
    ORD7 = 0b0110,
    ORD8 = 0b0111,
    ORD9 = 0b1000,
    ORD10 = 0b1001,
}

impl SampleRate {
    fn as_f32(self) -> f32 {
        match self {
            SampleRate::ORD1 => 0.0,
            SampleRate::ORD2 => 3.125,
            SampleRate::ORD3 => 6.25,
            SampleRate::ORD4 => 12.5,
            SampleRate::ORD5 => 25.0,
            SampleRate::ORD6 => 50.0,
            SampleRate::ORD7 => 100.0,
            SampleRate::ORD8 => 400.0,
            SampleRate::ORD9 => 800.0,
            SampleRate::ORD10 => 1600.0,
        }
    }
}

/// Error type wrapping all possible Spi and Output errors
#[derive(Debug)]
pub enum Error<SPIE, CSE> {
    SPI(SPIE),
    CS(CSE),
    WouldBlock,
}

/// Structure representing a LIS3DSH device, an embedded accelerometer
#[derive(Copy, Clone)]
pub struct LIS3DSH<Spi, Cs> {
    spi: Spi,
    cs: Cs,
    scale: Scale,
    sample_rate: SampleRate,
}

unsafe impl<SPI, CS> Send for LIS3DSH<SPI, CS>
where
    SPI: Send,
    CS: Send,
{
}

unsafe impl<SPI, CS> Sync for LIS3DSH<SPI, CS>
where
    SPI: Sync,
    CS: Sync,
{
}

impl<SPI, CS, SPIE, CSE> LIS3DSH<SPI, CS>
where
    SPI: Write<u8, Error = SPIE> + Transfer<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    SPIE: core::fmt::Debug,
    CSE: core::fmt::Debug,
{
    /// Create a new LIS3DSH struct. Chip select is set high and
    /// x, y, and z axes are activated.
    ///
    /// Default ORD (output data rate) is 100hz
    pub fn new(spi: SPI, cs: CS) -> Result<Self, Error<SPIE, CSE>> {
        let scale = Scale::G2;
        let sample_rate = SampleRate::ORD7;
        let mut accelerometer = LIS3DSH {
            spi,
            cs,
            scale,
            sample_rate,
        };

        accelerometer.cs.set_high().map_err(Error::CS)?;

        accelerometer.write_register(Register::CTRL_REG4, 0x67)?;

        Ok(accelerometer)
    }

    /// Free all owned peripherals
    pub fn free(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }

    /// Read a value from an arbitrary register of the LIS3DSH
    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<SPIE, CSE>> {
        let mut buffer = [register as u8 | 0x80, 0x00];
        self.cs.set_low().map_err(Error::CS)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPI)?;
        self.cs.set_low().map_err(Error::CS)?;

        Ok(buffer[1])
    }

    /// Write a value to an arbitray register of the LIS3DSH
    pub fn write_register(&mut self, register: Register, word: u8) -> Result<(), Error<SPIE, CSE>> {
        let buffer = [register as u8, word];
        self.cs.set_low().map_err(Error::CS)?;
        self.spi.write(&buffer).map_err(Error::SPI)?;
        self.cs.set_low().map_err(Error::CS)?;

        Ok(())
    }

    pub fn set_scale(&mut self, scale: Scale) -> Result<(), Error<SPIE, CSE>> {
        let ctrl_reg4 = self.read_register(Register::CTRL_REG4)?;
        self.write_register(Register::CTRL_REG4, ctrl_reg4 & !0b111 | scale as u8)?;
        self.scale = scale;
        Ok(())
    }

    pub fn set_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), Error<SPIE, CSE>> {
        let ctrl_reg4 = self.read_register(Register::CTRL_REG4)?;
        self.write_register(
            Register::CTRL_REG4,
            ctrl_reg4 & !0b1111 << 4 | (sample_rate as u8) << 4,
        )?;
        self.sample_rate = sample_rate;
        Ok(())
    }

    /// Activates an active high interrupt on INT1 when new data is ready
    pub fn enable_dr_interrupt(&mut self) -> Result<(), Error<SPIE, CSE>> {
        let register = self.read_register(Register::CTRL_REG3)?;
        self.write_register(Register::CTRL_REG3, register | 0xC8)
    }

    /// Calibrates the LIS3DSH device
    ///
    /// * `accleration` - A measurement of accleration during zero accleration
    pub fn calibrate_acceleration(&mut self, accleration: I16x3) -> Result<(), Error<SPIE, CSE>> {
        self.write_register(Register::OFF_X, cast(accleration.x / 32))?;
        self.write_register(Register::OFF_Y, cast(accleration.y / 32))?;
        self.write_register(Register::OFF_Z, cast(accleration.z / 32))?;

        Ok(())
    }
}

impl<SPI, CS, SPIE, CSE> Accelerometer<I16x3> for LIS3DSH<SPI, CS>
where
    SPI: Write<u8, Error = SPIE> + Transfer<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    SPIE: core::fmt::Debug,
    CSE: core::fmt::Debug,
{
    type Error = Error<SPIE, CSE>;

    /// Read normalized acceleration data
    fn accel_norm(&mut self) -> Result<F32x3, Error<SPIE, CSE>> {
        let acceleration = self.accel_raw()?;

        let x = (acceleration.x as f32 / core::i8::MAX as f32) * self.scale.as_f32();
        let y = (acceleration.y as f32 / core::i8::MAX as f32) * self.scale.as_f32();
        let z = (acceleration.z as f32 / core::i8::MAX as f32) * self.scale.as_f32();

        Ok(F32x3 { x, y, z })
    }

    /// Read raw acceleration data
    fn accel_raw(&mut self) -> Result<I16x3, Error<SPIE, CSE>> {
        let status = self.read_register(Register::STATUS)?;

        if status & (1 << 3) == 0 {
            return Err(Error::WouldBlock);
        }

        let out_x_l = self.read_register(Register::OUT_X_L)?;
        let out_x_h = self.read_register(Register::OUT_X_H)?;
        let out_y_l = self.read_register(Register::OUT_Y_L)?;
        let out_y_h = self.read_register(Register::OUT_Y_H)?;
        let out_z_l = self.read_register(Register::OUT_Z_L)?;
        let out_z_h = self.read_register(Register::OUT_Z_H)?;

        let x = (i16::from(out_x_h)) << 8 | i16::from(out_x_l);
        let y = (i16::from(out_y_h)) << 8 | i16::from(out_y_l);
        let z = (i16::from(out_z_h)) << 8 | i16::from(out_z_l);

        Ok(I16x3::new(x, y, z))
    }

    /// Read sample rate of accelerometer
    fn sample_rate(&mut self) -> Result<f32, Error<SPIE, CSE>> {
        Ok(self.sample_rate.as_f32())
    }
}

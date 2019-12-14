#![no_std]
#![allow(non_camel_case_types)]

use embedded_hal as hal;
use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

pub use accelerometer::{Accelerometer, I16x3};

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

// Error type wrapping all possible Spi and Output errors
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
        let mut accelerometer = LIS3DSH { spi, cs };

        accelerometer.cs.set_high().map_err(Error::CS)?;

        accelerometer.write_register(Register::CTRL_REG4, 0x67)?;

        Ok(accelerometer)
    }

    /// Free all owned peripherals
    pub fn free(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }

    /// Activates an active high interrupt on INT1 when new data is ready
    pub fn enable_dr_interrupt(&mut self) -> Result<(), Error<SPIE, CSE>> {
        let register = self.read_register(Register::CTRL_REG3)?;
        self.write_register(Register::CTRL_REG3, register | 0xC8)
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

    /// Read acceleration data
    fn acceleration(&mut self) -> Result<I16x3, Error<SPIE, CSE>> {
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
}

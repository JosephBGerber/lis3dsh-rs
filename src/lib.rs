#![no_std]
#![allow(dead_code)]
// ![deny(missing_docs)]

#[macro_use]
extern crate bitflags;

use embedded_hal as hal;

use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

pub struct Measurement {
    x: u16,
    y: u16,
    z: u16,
}

pub const STATUS_ADDR: u8 = 0x27;

bitflags! {
    struct STATUS: u8 {
        const XDA = 0b0000_0001;
        const YDA = 0b0000_0010;
        const ZDA = 0b0000_0100;
        const ZYXDA = 0b0000_1000;
        const XOR = 0b0001_0000;
        const YOR = 0b0010_0000;
        const ZOR = 0b0100_0000;
        const ZYXOR = 0b1000_0000;
    }
}

pub const OUT_X_L: u8 = 0x28;
pub const OUT_X_H: u8 = 0x29;
pub const OUT_Y_L: u8 = 0x2A;
pub const OUT_Y_H: u8 = 0x2B;
pub const OUT_Z_L: u8 = 0x2C;
pub const OUT_Z_H: u8 = 0x2D;

#[derive(Debug)]
pub enum Error<SpiE, CsE> {
    Spi(SpiE),
    Cs(CsE),
}

struct LIS3DSH<Spi, Cs> {
    spi: Spi,
    cs: Cs,
}

impl<Spi, Cs, SpiE, CsE> LIS3DSH<Spi, Cs>
where
    Spi: Write<u8, Error = SpiE> + Transfer<u8, Error = SpiE>,
    Cs: OutputPin<Error = CsE>,
{
    pub fn new(spi: Spi, cs: Cs) -> Self {
        LIS3DSH { spi, cs }
    }

    pub fn free(self) -> (Spi, Cs) {
        (self.spi, self.cs)
    }

    pub fn read_register(&mut self, address: u8) -> Result<u8, Error<SpiE, CsE>> {
        let data = [address];

        if let Err(e) = self.cs.set_low() {
            return Err(Error::Cs(e));
        };
        if let Err(e) = self.spi.transfer(&mut [address]) {
            return Err(Error::Spi(e));
        }
        if let Err(e) = self.cs.set_high() {
            return Err(Error::Cs(e));
        };

        Ok(data[0])
    }

    pub fn write_register(&mut self, address: u8, value: u8) -> Result<(), Error<SpiE, CsE>> {
        let data = [address | 0x80, value];

        if let Err(e) = self.cs.set_low() {
            return Err(Error::Cs(e));
        };
        if let Err(e) = self.spi.write(&data) {
            return Err(Error::Spi(e));
        }
        if let Err(e) = self.cs.set_high() {
            return Err(Error::Cs(e));
        };

        Ok(())
    }

    pub fn read_measurement(&mut self) -> Result<Measurement, Error<SpiE, CsE>> {
        loop {
            let status = match self.read_register(STATUS_ADDR) {
                Ok(value) => value,
                Err(e) => return Err(e),
            };

            if STATUS::from_bits(status).unwrap().contains(STATUS::ZYXDA) {
                break;
            }
        }

        let out_x_l = self.read_register(OUT_X_L)?;
        let out_x_h = self.read_register(OUT_X_H)?;
        let out_y_l = self.read_register(OUT_Y_L)?;
        let out_y_h = self.read_register(OUT_Y_H)?;
        let out_z_l = self.read_register(OUT_Z_L)?;
        let out_z_h = self.read_register(OUT_Z_H)?;

        let x = (u16::from(out_x_h)) << 8 | u16::from(out_x_l);
        let y = (u16::from(out_y_h)) << 8 | u16::from(out_y_l);
        let z = (u16::from(out_z_h)) << 8 | u16::from(out_z_l);

        Ok(Measurement { x, y, z })
    }
}

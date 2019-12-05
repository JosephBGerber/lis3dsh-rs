#![no_std]
#![allow(dead_code)]
// ![deny(missing_docs)]

#[macro_use]
use bitflags;

use embedded_hal as hal;

use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

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
}

//! Platform-agnostic LIS2DH12 accelerometer driver which uses I2C via
//! [embedded-hal] and implements the [`Accelerometer` trait][trait]
//! from the `accelerometer` crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//!

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

mod reg;

use core::fmt::Debug;

#[cfg(feature = "out_f32")]
pub use accelerometer::F32x3;
pub use accelerometer::{Accelerometer, Error, ErrorKind, I16x3};
use cast::u16;
#[cfg(feature = "out_f32")]
use cast::{f32, i16};
use embedded_hal as hal;
use hal::blocking::i2c::{Write, WriteRead};

use crate::reg::*;
pub use crate::reg::{FifoMode, FullScale, Mode, Odr};

/// Possible slave addresses
pub enum SlaveAddr {
    /// Default slave address
    Default,
    /// Alternative slave address providing bit value for `A0`
    Alternative(bool),
}

impl SlaveAddr {
    fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => I2C_SAD,
            SlaveAddr::Alternative(a0) => I2C_SAD | a0 as u8,
        }
    }
}

/// Data status structure,
/// decoded from STATUS_REG register
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,
    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),
    /// ZYXDA bit
    pub zyxda: bool,
    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

/// `LIS2DH12` driver
pub struct Lis2dh12<I2C> {
    /// The concrete I²C device implementation
    i2c: I2C,
    /// The I²C device slave address
    addr: u8,
    /// Current full-scale
    fs: FullScale,
}

impl<I2C, E> Lis2dh12<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Create a new `LIS2DH12` driver from the given `I2C` peripheral
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<E>> {
        let mut dev = Self {
            i2c,
            addr: addr.addr(),
            fs: FullScale::G2,
        };

        // Ensure we have the correct device ID
        if dev.get_device_id()? != DEVICE_ID {
            ErrorKind::Device.err()?;
        }

        Ok(dev)
    }

    /// Destroy driver instance, return `I2C` bus instance
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// `WHO_AM_I` register
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(Register::WHO_AM_I).map_err(Into::into)
    }

    /// Operating mode selection,
    /// `CTRL_REG1`: `LPen` bit,
    /// `CTRL_REG4`: `HR` bit
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::LowPower => {
                self.reg_reset_bits(Register::CTRL_REG4, HR)?;
                self.reg_set_bits(Register::CTRL_REG1, LPen)?;
            }
            Mode::Normal => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen)?;
                self.reg_reset_bits(Register::CTRL_REG4, HR)?;
            }
            Mode::HighResolution => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen)?;
                self.reg_set_bits(Register::CTRL_REG4, HR)?;
            }
        }
        Ok(())
    }

    /// Data rate selection,
    /// `CTRL_REG1`: `ODR`
    pub fn set_odr(&mut self, odr: Odr) -> Result<(), Error<E>> {
        self.modify_reg(Register::CTRL_REG1, |v| {
            (v & !ODR_MASK) | ((odr as u8) << 4)
        })?;
        // By design, when the device from high-resolution configuration (HR) is set to power-down mode (PD),
        // it is recommended to read register REFERENCE (26h) for a complete reset of the filtering block
        // before switching to normal/high-performance mode again.
        if let Odr::PowerDown = odr {
            self.get_ref()?;
        }
        Ok(())
    }

    /// X,Y,Z-axis enable,
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    pub fn enable_axis(&mut self, (x, y, z): (bool, bool, bool)) -> Result<(), Error<E>> {
        self.modify_reg(Register::CTRL_REG1, |mut v| {
            v &= !(Xen | Yen | Zen); // disable all axises
            if x {
                v |= Xen;
            }
            if y {
                v |= Yen;
            }
            if z {
                v |= Zen;
            }
            v
        })?;
        Ok(())
    }

    /// Block data update,
    /// `CTRL_REG4`: `BDU`
    pub fn set_bdu(&mut self, bdu: bool) -> Result<(), Error<E>> {
        self.reg_xset_bits(Register::CTRL_REG4, BDU, bdu)?;
        Ok(())
    }

    /// Full-scale selection,
    /// `CTRL_REG4`: `FS`
    pub fn set_fs(&mut self, fs: FullScale) -> Result<(), Error<E>> {
        self.modify_reg(Register::CTRL_REG4, |v| (v & !FS_MASK) | ((fs as u8) << 4))?;
        self.fs = fs;
        Ok(())
    }

    /// FIFO enable,
    /// `CTRL_REG5`: `FIFO_EN`
    pub fn enable_fifo(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.reg_xset_bits(Register::CTRL_REG5, FIFO_EN, enable)?;
        Ok(())
    }

    /// Data status,
    /// `STATUS_REG`: as
    /// DataStatus {zyxor: `ZYXOR`, xyzor: (`XOR`, `YOR`, `ZOR`), zyxda: `ZYXDA`, xyzda: (`XDA`, `YDA`, `ZDA`)}
    pub fn get_status(&mut self) -> Result<DataStatus, Error<E>> {
        let reg = self.read_reg(Register::STATUS_REG)?;
        Ok(DataStatus {
            zyxor: (reg & ZYXOR) != 0,
            xyzor: ((reg & XOR) != 0, (reg & YOR) != 0, (reg & ZOR) != 0),
            zyxda: (reg & ZYXDA) != 0,
            xyzda: ((reg & XDA) != 0, (reg & YDA) != 0, (reg & ZDA) != 0),
        })
    }

    /// FIFO mode selection,
    /// `FIFO_CTRL_REG`: `FM`
    pub fn set_fm(&mut self, fm: FifoMode) -> Result<(), Error<E>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FM_MASK) | ((fm as u8) << 6)
        })?;
        Ok(())
    }

    /// FIFO threshold,
    /// `FIFO_CTRL_REG`: `FTH`
    pub fn set_fth(&mut self, fth: u8) -> Result<(), Error<E>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FTH_MASK) | (fth & FTH_MASK)
        })?;
        Ok(())
    }

    /// Temperature sensor enable,
    /// `TEMP_CFG_REG`: `TEMP_EN`,
    /// the `BDU` bit in `CTRL_REG4` is also set
    pub fn enable_temp(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.reg_xset_bits(Register::TEMP_CFG_REG, TEMP_EN, enable)?;
        if enable {
            // enable block data update (required for temp reading)
            self.reg_set_bits(Register::CTRL_REG4, BDU)?;
        }
        Ok(())
    }

    /// Temperature data status,
    /// `STATUS_REG_AUX`: `TOR` - Temperature data overrun,
    ///                   `TDA` - Temperature new data available
    pub fn get_temp_status(&mut self) -> Result<(bool, bool), Error<E>> {
        let reg = self.read_reg(Register::STATUS_REG_AUX)?;
        Ok(((reg & TOR) != 0, (reg & TDA) != 0))
    }

    /// Temperature sensor data,
    /// `OUT_TEMP_H`, `OUT_TEMP_L`
    pub fn get_temp_out(&mut self) -> Result<(i8, u8), Error<E>> {
        let mut buf = [0u8; 2];
        self.read_regs(Register::OUT_TEMP_L, &mut buf)?;
        Ok((buf[1] as i8, buf[0]))
    }

    /// Temperature sensor data as float,
    /// `OUT_TEMP_H`, `OUT_TEMP_L` converted to `f32`
    #[cfg(feature = "out_f32")]
    pub fn get_temp_outf(&mut self) -> Result<(f32), Error<E>> {
        let (out_h, out_l) = self.get_temp_out()?;
        // 10-bit resolution
        let value = (i16(out_h) << 2) | i16(out_l >> 6);
        Ok(f32(value) * 0.25)
    }

    /// `REFERENCE` register
    pub fn get_ref(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(Register::REFERENCE).map_err(Into::into)
    }

    #[allow(dead_code)]
    fn dump_regs<W>(&mut self, w: &mut W) -> Result<(), Error<E>>
    where
        W: core::fmt::Write,
    {
        writeln!(
            w,
            "CTRL_REG1 = {:#010b}",
            self.read_reg(Register::CTRL_REG1)?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG4 = {:#010b}",
            self.read_reg(Register::CTRL_REG4)?
        )
        .unwrap();
        Ok(())
    }

    #[inline]
    fn read_reg(&mut self, reg: Register) -> Result<u8, E> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[reg.addr()], &mut buf)?;
        Ok(buf[0])
    }

    #[inline]
    fn read_regs(&mut self, reg: Register, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c
            .write_read(self.addr, &[reg.addr() | I2C_SUB_MULTI], buffer)
    }

    #[inline]
    fn write_reg(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.i2c.write(self.addr, &[reg.addr(), val])
    }

    #[inline]
    fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg)?;
        self.write_reg(reg, f(r))?;
        Ok(())
    }

    #[inline]
    fn reg_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), E> {
        self.modify_reg(reg, |v| v | bits)
    }

    #[inline]
    fn reg_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), E> {
        self.modify_reg(reg, |v| v & !bits)
    }

    #[inline]
    fn reg_xset_bits(&mut self, reg: Register, bits: u8, set: bool) -> Result<(), E> {
        if set {
            self.reg_set_bits(reg, bits)
        } else {
            self.reg_reset_bits(reg, bits)
        }
    }
}

impl<I2C, E> Accelerometer<I16x3> for Lis2dh12<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get acceleration reading from the accelerometer
    fn acceleration(&mut self) -> Result<I16x3, Error<E>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_L, &mut buf)?;

        Ok(I16x3::new(
            (u16(buf[0]) + (u16(buf[1]) << 8)) as i16,
            (u16(buf[2]) + (u16(buf[3]) << 8)) as i16,
            (u16(buf[4]) + (u16(buf[5]) << 8)) as i16,
        ))
    }
}

#[cfg(feature = "out_f32")]
impl<I2C, E> Accelerometer<F32x3> for Lis2dh12<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get acceleration reading from the accelerometer
    fn acceleration(&mut self) -> Result<F32x3, Error<E>> {
        let acc: I16x3 = self.acceleration()?;

        Ok(F32x3::new(
            self.fs.convert_i16tof32(acc.x),
            self.fs.convert_i16tof32(acc.y),
            self.fs.convert_i16tof32(acc.z),
        ))
    }
}

use core::marker::PhantomData;

use cast::u16;
#[cfg(feature = "out_f32")]
use cast::{f32, i16};
#[cfg(feature = "out_f32")]
use num_traits::FromPrimitive;

use super::{I2c, bisync, only_async, only_sync};
#[cfg(feature = "out_f32")]
use crate::F32x3;
use crate::reg::*;
use crate::{AddrDetectionError, DataStatus, Error, ErrorKind, I16x3, SlaveAddr, reg};
// Only used by the sync-only `RawAccelerometer`/`Accelerometer` impls below, so these
// imports must vanish along with them in the async build (else it's a deny(warnings) error).
#[cfg(feature = "out_f32")]
#[only_sync]
use crate::Accelerometer;
#[only_sync]
use crate::RawAccelerometer;

/// `LIS2DH12` driver
pub struct Lis2dh12<I2C> {
    /// The concrete I²C device implementation
    i2c: I2C,
    /// The I²C device slave address
    addr: u8,
    /// Current full-scale
    #[cfg(feature = "out_f32")]
    fs: FullScale,
}

/// This function tries to detect the I2C address of the device by scanning all possible I2C
/// addresses.
#[bisync]
pub async fn detect_i2c_addr<I2C: I2c>(
    i2c: &mut I2C,
) -> Result<SlaveAddr, AddrDetectionError<I2C::Error>> {
    let mut buf = [0u8];
    let write_buf = &[reg::Register::WHO_AM_I.addr()];
    match i2c
        .write_read(reg::I2C_SAD | 0b1, write_buf, &mut buf)
        .await
    {
        Ok(_) => {
            if buf[0] == reg::DEVICE_ID {
                return Ok(SlaveAddr::Alternative(true));
            }
            Err(AddrDetectionError::InvalidDeviceId)
        }
        Err(_) => {
            let result = i2c.write_read(reg::I2C_SAD, write_buf, &mut buf).await;
            if result.is_ok() {
                if buf[0] == reg::DEVICE_ID {
                    return Ok(SlaveAddr::Default);
                } else {
                    return Err(AddrDetectionError::InvalidDeviceId);
                }
            }
            Err(result.unwrap_err().into())
        }
    }
}

/// Interrupt setting and status
pub struct Int<'a, REG, I2C> {
    dev: &'a mut Lis2dh12<I2C>,
    reg: PhantomData<REG>,
}

#[bisync]
impl<I2C: I2c> Lis2dh12<I2C> {
    /// Create a new `LIS2DH12` driver from the given `I2C` peripheral
    pub async fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<I2C::Error>> {
        let mut dev = Self {
            i2c,
            addr: addr.addr(),
            #[cfg(feature = "out_f32")]
            fs: FullScale::G2,
        };

        // Ensure we have the correct device ID
        if dev.get_device_id().await? != DEVICE_ID {
            ErrorKind::Device.err()?;
        }

        Ok(dev)
    }

    /// Destroy driver instance, return `I2C` bus instance
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// `WHO_AM_I` register
    pub async fn get_device_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(Register::WHO_AM_I).await.map_err(Into::into)
    }

    /// Operating mode selection,
    /// `CTRL_REG1`: `LPen` bit,
    /// `CTRL_REG4`: `HR` bit
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<I2C::Error>> {
        match mode {
            Mode::LowPower => {
                self.reg_reset_bits(Register::CTRL_REG4, HR).await?;
                self.reg_set_bits(Register::CTRL_REG1, LPen).await?;
            }
            Mode::Normal => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen).await?;
                self.reg_reset_bits(Register::CTRL_REG4, HR).await?;
            }
            Mode::HighResolution => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen).await?;
                self.reg_set_bits(Register::CTRL_REG4, HR).await?;
            }
        }
        Ok(())
    }

    /// Data rate selection,
    /// `CTRL_REG1`: `ODR`
    pub async fn set_odr(&mut self, odr: Odr) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CTRL_REG1, |v| {
            (v & !ODR_MASK) | ((odr as u8) << 4)
        })
        .await?;
        // By design, when the device from high-resolution configuration (HR) is set to power-down mode (PD),
        // it is recommended to read register REFERENCE (26h) for a complete reset of the filtering block
        // before switching to normal/high-performance mode again.
        if let Odr::PowerDown = odr {
            self.get_ref().await?;
        }
        Ok(())
    }

    /// X,Y,Z-axis enable,
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    pub async fn enable_axis(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CTRL_REG1, |mut v| {
            v &= !(Xen | Yen | Zen); // disable all axises
            v |= if x { Xen } else { 0 };
            v |= if y { Yen } else { 0 };
            v |= if z { Zen } else { 0 };
            v
        })
        .await?;
        Ok(())
    }

    /// Enable high-pass filter for CLICK/IA2/IA1
    pub async fn enable_hp_filter(
        &mut self,
        click: bool,
        ia2: bool,
        ia1: bool,
    ) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CTRL_REG2, |mut v| {
            v &= !(HPCLICK | HP_IA2 | HP_IA1); // disable all filters
            v |= if click { HPCLICK } else { 0 };
            v |= if ia2 { HP_IA2 } else { 0 };
            v |= if ia1 { HP_IA1 } else { 0 };
            v
        })
        .await?;
        Ok(())
    }

    /// `CLICK` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_CLICK`
    pub async fn enable_i1_click(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_CLICK, enable)
            .await?;
        Ok(())
    }

    /// `IA1` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_IA1`
    pub async fn enable_i1_ia1(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_IA1, enable)
            .await?;
        Ok(())
    }

    /// `IA2` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_IA2`
    pub async fn enable_i1_ia2(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_IA2, enable)
            .await?;
        Ok(())
    }

    /// `ZYXDA` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I2_ZYXDA`
    pub async fn enable_i1_zyxda(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_ZYXDA, enable)
            .await?;
        Ok(())
    }

    /// FIFO watermark on `INT1` pin,
    /// `CTRL_REG3`: `I2_ZYXDA`
    pub async fn enable_i1_wtm(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_WTM, enable)
            .await?;
        Ok(())
    }

    /// Get the amount of samples currently stored in the FIFO queue
    pub async fn get_stored_samples(&mut self) -> Result<u8, Error<I2C::Error>> {
        let value = self.read_reg(Register::FIFO_SRC_REG).await?;
        Ok(value & FSS)
    }

    /// FIFO overrun on `INT1` pin,
    /// `CTRL_REG3`: `I1_OVERRUN`
    pub async fn enable_i1_overrun(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_OVERRUN, enable)
            .await?;
        Ok(())
    }

    /// Block data update,
    /// `CTRL_REG4`: `BDU`
    pub async fn set_bdu(&mut self, bdu: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, BDU, bdu).await?;
        Ok(())
    }

    /// Full-scale selection,
    /// `CTRL_REG4`: `FS`
    pub async fn set_fs(&mut self, fs: FullScale) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CTRL_REG4, |v| (v & !FS_MASK) | ((fs as u8) << 4))
            .await?;
        #[cfg(feature = "out_f32")]
        {
            self.fs = fs;
        }
        Ok(())
    }

    /// Reboot memory content,
    /// `CTRL_REG5`: `BOOT`
    pub async fn reboot(&mut self, reboot: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, BOOT, reboot)
            .await?;
        Ok(())
    }

    /// In boot,
    /// `CTRL_REG5`: `BOOT`
    pub async fn in_boot(&mut self) -> Result<bool, Error<I2C::Error>> {
        let reg = self.read_reg(Register::CTRL_REG5).await?;
        Ok((reg & BOOT) != 0)
    }

    /// FIFO enable,
    /// `CTRL_REG5`: `FIFO_EN`
    pub async fn enable_fifo(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, FIFO_EN, enable)
            .await?;
        Ok(())
    }

    /// Latch interrupt request on INT1_SRC (31h),
    /// with INT1_SRC (31h) register cleared by reading INT1_SRC (31h) itself,
    /// `CTRL_REG5`: `LIR_INT1`
    pub async fn enable_lir_int1(&mut self, latch: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, LIR_INT1, latch)
            .await?;
        Ok(())
    }

    /// 4D enable: 4D detection is enabled on INT1 pin
    /// when 6D bit on INT1_CFG (30h) is set to 1,
    /// `CTRL_REG5`: `D4D_INT1`
    pub async fn enable_d4d_int1(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, D4D_INT1, enable)
            .await?;
        Ok(())
    }

    /// Latch interrupt request on INT2_SRC (35h) register,
    /// with INT2_SRC (35h) register cleared by reading INT2_SRC (35h) itself,
    /// `CTRL_REG5`: `LIR_INT2`
    pub async fn enable_lir_int2(&mut self, latch: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, LIR_INT2, latch)
            .await?;
        Ok(())
    }

    /// 4D enable: 4D detection is enabled on INT2 pin
    /// when 6D bit on INT2_CFG (34h) is set to 1,
    /// `CTRL_REG5`: `D4D_INT2`
    pub async fn enable_d4d_int2(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, D4D_INT2, enable)
            .await?;
        Ok(())
    }

    /// `CLICK` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_CLICK`
    pub async fn enable_i2_click(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_CLICK, enable)
            .await?;
        Ok(())
    }

    /// `IA1` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_IA1`
    pub async fn enable_i2_ia1(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_IA1, enable)
            .await?;
        Ok(())
    }

    /// `IA2` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_IA2`
    pub async fn enable_i2_ia2(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_IA2, enable)
            .await?;
        Ok(())
    }

    /// Boot interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_BOOT`
    pub async fn enable_i2_boot(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_BOOT, enable)
            .await?;
        Ok(())
    }

    /// Activity interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_ACT`
    pub async fn enable_i2_act(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_ACT, enable)
            .await?;
        Ok(())
    }

    /// INT1/INT2 pin polarity,
    /// `CTRL_REG6`: `INT_POLARITY`
    pub async fn set_int_polarity(&mut self, active_low: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, INT_POLARITY, active_low)
            .await?;
        Ok(())
    }

    /// Data status,
    /// `STATUS_REG`: as
    /// DataStatus {zyxor: `ZYXOR`, xyzor: (`XOR`, `YOR`, `ZOR`), zyxda: `ZYXDA`, xyzda: (`XDA`, `YDA`, `ZDA`)}
    pub async fn get_status(&mut self) -> Result<DataStatus, Error<I2C::Error>> {
        let reg = self.read_reg(Register::STATUS_REG).await?;
        Ok(DataStatus {
            zyxor: (reg & ZYXOR) != 0,
            xyzor: ((reg & XOR) != 0, (reg & YOR) != 0, (reg & ZOR) != 0),
            zyxda: (reg & ZYXDA) != 0,
            xyzda: ((reg & XDA) != 0, (reg & YDA) != 0, (reg & ZDA) != 0),
        })
    }

    /// FIFO mode selection,
    /// `FIFO_CTRL_REG`: `FM`
    pub async fn set_fm(&mut self, fm: FifoMode) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FM_MASK) | ((fm as u8) << 6)
        })
        .await?;
        Ok(())
    }

    /// FIFO threshold,
    /// `FIFO_CTRL_REG`: `FTH`
    pub async fn set_fth(&mut self, fth: u8) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FTH_MASK) | (fth & FTH_MASK)
        })
        .await?;
        Ok(())
    }

    /// Disable click interrupt,
    /// `CLICK_CFG` clean all bits
    pub async fn disable_click(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::CLICK_CFG, 0x00).await?;
        Ok(())
    }

    /// Enable interrupt double-click on X,Y,Z axis,
    /// `CLICK_CFG`: `XD`, `YD`, `ZD`
    pub async fn enable_double_click(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CLICK_CFG, |mut v| {
            v &= !(XD | YD | ZD); // disable all axises
            v |= if x { XD } else { 0 };
            v |= if y { YD } else { 0 };
            v |= if z { ZD } else { 0 };
            v
        })
        .await?;
        Ok(())
    }

    /// Enable interrupt single-click on X,Y,Z axis,
    /// `CLICK_CFG`: `XS`, `YS`, `ZS`
    pub async fn enable_single_click(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<I2C::Error>> {
        self.modify_reg(Register::CLICK_CFG, |mut v| {
            v &= !(XS | YS | ZS); // disable all axises
            v |= if x { XS } else { 0 };
            v |= if y { YS } else { 0 };
            v |= if z { ZS } else { 0 };
            v
        })
        .await?;
        Ok(())
    }

    /// Click source,
    /// `CLICK_SRC` decoded as ((`DClick`, `SClick`), `Sign`, (`X`, `Y`, `Z`))
    #[allow(clippy::type_complexity)]
    pub async fn get_click_src(
        &mut self,
    ) -> Result<Option<((bool, bool), bool, (bool, bool, bool))>, Error<I2C::Error>> {
        let reg = self.read_reg(Register::CLICK_SRC).await?;
        if (reg & IA) != 0 {
            Ok(Some((
                ((reg & DClick) != 0, (reg & SClick) != 0),
                (reg & Sign) != 0,
                ((reg & X) != 0, (reg & Y) != 0, (reg & Z) != 0),
            )))
        } else {
            Ok(None)
        }
    }

    /// If the LIR_Click bit is not set, the interrupt is kept high
    /// for the duration of the latency window.
    /// If the LIR_Click bit is set, the interrupt is kept high
    /// until the CLICK_SRC (39h) register is read.
    /// `CLICK_THS`: `LIR_Click`
    pub async fn enable_lir_click(&mut self, latch: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CLICK_THS, LIR_Click, latch)
            .await?;
        Ok(())
    }

    /// Click threshold,
    /// `CLICK_THS`: `Ths`
    pub async fn set_click_ths(&mut self, ths: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::CLICK_THS, ths & THS_MASK).await?;
        Ok(())
    }

    /// Click threshold as f32,
    /// `CLICK_THS`: `Ths`
    #[cfg(feature = "out_f32")]
    pub async fn set_click_thsf(&mut self, ths: f32) -> Result<(), Error<I2C::Error>> {
        self.set_click_ths(self.fs.convert_ths_f32tou8(ths)).await
    }

    /// Click time limit,
    /// `TIME_LIMIT`: `TLI`
    pub async fn set_time_limit(&mut self, tli: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::TIME_LIMIT, tli & TLI_MASK).await?;
        Ok(())
    }

    /// Click time latency,
    /// `TIME_LATENCY`: `TLA`
    pub async fn set_time_latency(&mut self, tla: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::TIME_LATENCY, tla).await?;
        Ok(())
    }

    /// Click time window,
    /// `TIME_WINDOW`: `TW`
    pub async fn set_time_window(&mut self, tw: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::TIME_WINDOW, tw).await?;
        Ok(())
    }

    /// Sleep-to-wake, return-to-sleep activation threshold in low-power mode,
    /// `ACT_THS`: `Acth`
    pub async fn set_act_ths(&mut self, ths: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::ACT_THS, ths & Acth_MASK).await?;
        Ok(())
    }

    /// Sleep-to-wake, return-to-sleep activation threshold as f32,
    /// `ACT_THS`: `Acth`
    #[cfg(feature = "out_f32")]
    pub async fn set_act_thsf(&mut self, ths: f32) -> Result<(), Error<I2C::Error>> {
        self.set_act_ths(self.fs.convert_ths_f32tou8(ths)).await
    }

    /// Sleep-to-wake, return-to-sleep duration,
    /// `ACT_DUR`: `ActD`
    pub async fn set_act_dur(&mut self, d: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::ACT_DUR, d).await?;
        Ok(())
    }

    /// Temperature sensor enable,
    /// `TEMP_CFG_REG`: `TEMP_EN`,
    /// the `BDU` bit in `CTRL_REG4` is also set
    pub async fn enable_temp(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::TEMP_CFG_REG, TEMP_EN, enable)
            .await?;
        if enable {
            // enable block data update (required for temp reading)
            self.reg_set_bits(Register::CTRL_REG4, BDU).await?;
        }
        Ok(())
    }

    /// Temperature data status,
    /// `STATUS_REG_AUX`: `TOR` - Temperature data overrun,
    ///                   `TDA` - Temperature new data available
    pub async fn get_temp_status(&mut self) -> Result<(bool, bool), Error<I2C::Error>> {
        let reg = self.read_reg(Register::STATUS_REG_AUX).await?;
        Ok(((reg & TOR) != 0, (reg & TDA) != 0))
    }

    /// Temperature sensor data,
    /// `OUT_TEMP_H`, `OUT_TEMP_L`
    pub async fn get_temp_out(&mut self) -> Result<(i8, u8), Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(Register::OUT_TEMP_L, &mut buf).await?;
        Ok((buf[1] as i8, buf[0]))
    }

    /// Temperature sensor data as float,
    /// `OUT_TEMP_H`, `OUT_TEMP_L` converted to `f32`
    #[cfg(feature = "out_f32")]
    pub async fn get_temp_outf(&mut self) -> Result<f32, Error<I2C::Error>> {
        let (out_h, out_l) = self.get_temp_out().await?;
        // 10-bit resolution
        let value = (i16(out_h) << 8) | i16(out_l);
        Ok(25.0 + f32(value) / 256.0)
    }

    /// `REFERENCE` register
    pub async fn set_ref(&mut self, reference: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::REFERENCE, reference).await?;
        Ok(())
    }

    /// `REFERENCE` register
    pub async fn get_ref(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(Register::REFERENCE).await.map_err(Into::into)
    }

    /// INT1
    pub fn int1(&mut self) -> Int<'_, Int1Regs, I2C> {
        Int::new(self)
    }

    /// INT2
    pub fn int2(&mut self) -> Int<'_, Int2Regs, I2C> {
        Int::new(self)
    }

    /// Resets all registers to their default
    pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::CTRL_REG1, CTRL_REG1_DEFAULT)
            .await?;
        self.write_reg(Register::CTRL_REG2, CTRL_REG2_DEFAULT)
            .await?;
        self.write_reg(Register::CTRL_REG3, CTRL_REG3_DEFAULT)
            .await?;
        self.write_reg(Register::CTRL_REG4, CTRL_REG4_DEFAULT)
            .await?;
        self.write_reg(Register::CTRL_REG5, CTRL_REG5_DEFAULT)
            .await?;
        self.write_reg(Register::CTRL_REG6, CTRL_REG6_DEFAULT)
            .await?;
        self.write_reg(Register::INT1_CFG, INT_CFG_DEFAULT).await?;
        self.write_reg(Register::INT2_CFG, INT_CFG_DEFAULT).await?;
        self.write_reg(Register::INT1_THS, INT_THS_DEFAULT).await?;
        self.write_reg(Register::INT2_THS, INT_THS_DEFAULT).await?;
        Ok(())
    }

    /// Enable self test 0, this shouldn't be enabled at the same time as self test 1
    pub async fn enable_st0(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, ST0, enable).await?;
        Ok(())
    }

    /// Enable self test 1, this shouldn't be enabled at the same time as self test 0
    pub async fn enable_st1(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, ST1, enable).await?;
        Ok(())
    }

    /// Dump registers
    #[cfg(debug_assertions)]
    pub async fn dump_regs<W>(&mut self, w: &mut W) -> Result<(), Error<I2C::Error>>
    where
        W: core::fmt::Write,
    {
        writeln!(
            w,
            "CTRL_REG1 (20h) = {:#010b}",
            self.read_reg(Register::CTRL_REG1).await?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG3 (22h) = {:#010b}",
            self.read_reg(Register::CTRL_REG3).await?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG4 (23h) = {:#010b}",
            self.read_reg(Register::CTRL_REG4).await?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG5 (24h) = {:#010b}",
            self.read_reg(Register::CTRL_REG5).await?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG6 (25h) = {:#010b}",
            self.read_reg(Register::CTRL_REG6).await?
        )
        .unwrap();
        writeln!(
            w,
            "INT1_CFG (30h) = {:#010b}",
            self.read_reg(Register::INT1_CFG).await?
        )
        .unwrap();
        writeln!(
            w,
            "INT1_THS (32h) = {:#010b}",
            self.read_reg(Register::INT1_THS).await?
        )
        .unwrap();
        writeln!(
            w,
            "FIFO_SRC_REG (2Fh) = {:#010b}",
            self.read_reg(Register::FIFO_SRC_REG).await?
        )
        .unwrap();
        writeln!(
            w,
            "FIFO_CTRL_REG (2Fh) = {:#010b}",
            self.read_reg(Register::FIFO_CTRL_REG).await?
        )
        .unwrap();
        Ok(())
    }

    #[inline]
    async fn read_reg(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.addr, &[reg.addr()], &mut buf)
            .await?;
        Ok(buf[0])
    }

    #[inline]
    async fn read_regs(&mut self, reg: Register, buffer: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c
            .write_read(self.addr, &[reg.addr() | I2C_SUB_MULTI], buffer)
            .await
    }

    #[inline]
    async fn write_reg(&mut self, reg: Register, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg.addr(), val]).await
    }

    #[inline]
    async fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg).await?;
        self.write_reg(reg, f(r)).await?;
        Ok(())
    }

    #[inline]
    async fn reg_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), I2C::Error> {
        self.modify_reg(reg, |v| v | bits).await
    }

    #[inline]
    async fn reg_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), I2C::Error> {
        self.modify_reg(reg, |v| v & !bits).await
    }

    #[inline]
    async fn reg_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), I2C::Error> {
        if set {
            self.reg_set_bits(reg, bits).await
        } else {
            self.reg_reset_bits(reg, bits).await
        }
    }
}

#[only_async]
impl<I2C: I2c> Lis2dh12<I2C> {
    /// Get acceleration reading from the accelerometer
    pub async fn accel_raw(&mut self) -> Result<I16x3, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_L, &mut buf).await?;

        Ok(I16x3::new(
            (u16(buf[0]) + (u16(buf[1]) << 8)) as i16,
            (u16(buf[2]) + (u16(buf[3]) << 8)) as i16,
            (u16(buf[4]) + (u16(buf[5]) << 8)) as i16,
        ))
    }

    /// Get normalized ±g reading from the accelerometer
    #[cfg(feature = "out_f32")]
    pub async fn accel_norm(&mut self) -> Result<F32x3, Error<I2C::Error>> {
        let acc_raw: I16x3 = self.accel_raw().await?;

        Ok(F32x3::new(
            self.fs.convert_out_i16tof32(acc_raw.x),
            self.fs.convert_out_i16tof32(acc_raw.y),
            self.fs.convert_out_i16tof32(acc_raw.z),
        ))
    }

    /// Get sample rate of accelerometer in Hz
    #[cfg(feature = "out_f32")]
    pub async fn sample_rate(&mut self) -> Result<f32, Error<I2C::Error>> {
        let creg1 = self.read_reg(Register::CTRL_REG1).await?;
        let rate = match FromPrimitive::from_u8(creg1 >> 4) {
            Some(Odr::PowerDown) => 0.0,
            Some(Odr::Hz1) => 1.0,
            Some(Odr::Hz10) => 10.0,
            Some(Odr::Hz25) => 25.0,
            Some(Odr::Hz50) => 50.0,
            Some(Odr::Hz100) => 100.0,
            Some(Odr::Hz200) => 200.0,
            Some(Odr::Hz400) => 400.0,
            Some(Odr::HighRate0) => 1620.0,
            Some(Odr::HighRate1) => {
                if creg1 & LPen == 0 {
                    1344.0
                } else {
                    5376.0
                }
            }
            None => 0.0,
        };
        Ok(rate)
    }
}

#[only_sync]
impl<I2C: I2c> RawAccelerometer<I16x3> for Lis2dh12<I2C> {
    type Error = I2C::Error;

    /// Get acceleration reading from the accelerometer
    fn accel_raw(&mut self) -> Result<I16x3, Error<Self::Error>> {
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
#[only_sync]
impl<I2C: I2c> Accelerometer for Lis2dh12<I2C> {
    type Error = I2C::Error;

    /// Get normalized ±g reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        let acc_raw: I16x3 = self.accel_raw()?;

        Ok(F32x3::new(
            self.fs.convert_out_i16tof32(acc_raw.x),
            self.fs.convert_out_i16tof32(acc_raw.y),
            self.fs.convert_out_i16tof32(acc_raw.z),
        ))
    }

    /// Get sample rate of accelerometer in Hz
    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        let creg1 = self.read_reg(Register::CTRL_REG1)?;
        let rate = match FromPrimitive::from_u8(creg1 >> 4) {
            Some(Odr::PowerDown) => 0.0,
            Some(Odr::Hz1) => 1.0,
            Some(Odr::Hz10) => 10.0,
            Some(Odr::Hz25) => 25.0,
            Some(Odr::Hz50) => 50.0,
            Some(Odr::Hz100) => 100.0,
            Some(Odr::Hz200) => 200.0,
            Some(Odr::Hz400) => 400.0,
            Some(Odr::HighRate0) => 1620.0,
            Some(Odr::HighRate1) => {
                if creg1 & LPen == 0 {
                    1344.0
                } else {
                    5376.0
                }
            }
            None => 0.0,
        };
        Ok(rate)
    }
}

#[bisync]
impl<'a, REG, I2C: I2c> Int<'a, REG, I2C>
where
    REG: IntRegs,
{
    fn new(dev: &'a mut Lis2dh12<I2C>) -> Self {
        Self {
            dev,
            reg: PhantomData,
        }
    }

    /// Disable interrupt,
    /// `INTx_CFG` clean all bits
    pub async fn disable(&mut self) -> Result<(), Error<I2C::Error>> {
        self.dev.write_reg(REG::reg_cfg(), 0x00).await?;
        Ok(())
    }

    /// AOI-6D Interrupt mode,
    /// `INTx_CFG`: `AOI`, `6D`
    pub async fn set_mode(&mut self, mode: Aoi6d) -> Result<(), Error<I2C::Error>> {
        self.dev
            .modify_reg(REG::reg_cfg(), |v| (v & !AOI_6D_MASK) | ((mode as u8) << 6))
            .await?;
        Ok(())
    }

    /// X,Y,Z high event enable,
    /// `INTx_CFG`: `XHIE`, `YHIE`, `ZHIE`
    pub async fn enable_high(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<I2C::Error>> {
        self.dev
            .modify_reg(REG::reg_cfg(), |mut v| {
                v &= !(XHIE | YHIE | ZHIE); // disable all axises
                v |= if x { XHIE } else { 0 };
                v |= if y { YHIE } else { 0 };
                v |= if z { ZHIE } else { 0 };
                v
            })
            .await?;
        Ok(())
    }

    /// X,Y,Z low event enable,
    /// `INTx_CFG`: `XLIE`, `YLIE`, `ZLIE`
    pub async fn enable_low(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<I2C::Error>> {
        self.dev
            .modify_reg(REG::reg_cfg(), |mut v| {
                v &= !(XLIE | YLIE | ZLIE); // disable all axises
                v |= if x { XLIE } else { 0 };
                v |= if y { YLIE } else { 0 };
                v |= if z { ZLIE } else { 0 };
                v
            })
            .await?;
        Ok(())
    }

    /// Source,
    /// `INTx_SRC` decoded as ((`XH`, `XL`), (`YH`, `YL`), (`ZH`, `ZL`))
    #[allow(clippy::type_complexity)]
    pub async fn get_src(
        &mut self,
    ) -> Result<Option<((bool, bool), (bool, bool), (bool, bool))>, Error<I2C::Error>> {
        let reg = self.dev.read_reg(REG::reg_src()).await?;
        if (reg & IA) != 0 {
            Ok(Some((
                ((reg & XH) != 0, (reg & XL) != 0),
                ((reg & YH) != 0, (reg & YL) != 0),
                ((reg & ZH) != 0, (reg & ZL) != 0),
            )))
        } else {
            Ok(None)
        }
    }

    /// Threshold,
    /// `INTx_THS`: `THS`
    pub async fn set_ths(&mut self, ths: u8) -> Result<(), Error<I2C::Error>> {
        self.dev.write_reg(REG::reg_ths(), ths & THS_MASK).await?;
        Ok(())
    }

    /// Threshold as f32,
    /// `INTx_THS`: `THS`
    #[cfg(feature = "out_f32")]
    pub async fn set_thsf(&mut self, ths: f32) -> Result<(), Error<I2C::Error>> {
        self.set_ths(self.dev.fs.convert_ths_f32tou8(ths)).await
    }

    /// Duration,
    /// `INTx_DURATION`: `D`
    pub async fn set_duration(&mut self, d: u8) -> Result<(), Error<I2C::Error>> {
        self.dev.write_reg(REG::reg_duration(), d & D_MASK).await?;
        Ok(())
    }
}

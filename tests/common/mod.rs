pub use lis2dh12::{Lis2dh12, SlaveAddr};

use embedded_hal_mock as hal;
pub use hal::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};

pub const DEV_ADDR: u8 = 0b001_1000;

pub fn trans_who_am_i() -> I2cTrans {
    I2cTrans::write_read(DEV_ADDR, vec![0x0F], vec![0b0011_0011])
}

mod common;

use common::*;
use futures::executor::block_on;
use lis2dh12::asynch::Lis2dh12;

#[test]
fn create() {
    let trans = [trans_who_am_i()];
    let mock = I2cMock::new(&trans);
    block_on(async {
        let dev = Lis2dh12::new(mock, SlaveAddr::Default).await.unwrap();
        dev.destroy().done();
    });
}

#[test]
fn dev_id_get() {
    let trans = [trans_who_am_i(), trans_who_am_i()];
    let mock = I2cMock::new(&trans);
    block_on(async {
        let mut dev = Lis2dh12::new(mock, SlaveAddr::Alternative(false))
            .await
            .unwrap();
        let dev_id = dev.get_device_id().await.unwrap();
        assert_eq!(dev_id, 0b0011_0011);
        dev.destroy().done();
    });
}

#[test]
fn int1_disable() {
    let trans = [
        trans_who_am_i(),
        // write INT1_CFG: 0x00
        I2cTrans::write(DEV_ADDR, vec![0x30, 0x00]),
    ];
    let mock = I2cMock::new(&trans);
    block_on(async {
        let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).await.unwrap();
        dev.int1().disable().await.unwrap();
        dev.destroy().done();
    });
}

#[test]
fn accel_raw() {
    let trans = [
        trans_who_am_i(),
        // read OUT_X_L..OUT_Z_H
        I2cTrans::write_read(
            DEV_ADDR,
            vec![0x80 | 0x28],
            vec![0x01, 0x00, 0x02, 0x00, 0x03, 0x00],
        ),
    ];
    let mock = I2cMock::new(&trans);
    block_on(async {
        let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).await.unwrap();
        let acc = dev.accel_raw().await.unwrap();
        assert_eq!((acc.x, acc.y, acc.z), (1, 2, 3));
        dev.destroy().done();
    });
}

#[test]
fn temp_enable() {
    let trans = [
        trans_who_am_i(),
        // read TEMP_CFG_REG
        I2cTrans::write_read(DEV_ADDR, vec![0x1F], vec![0b0000_0000]),
        // write TEMP_CFG_REG: TEMP_EN
        I2cTrans::write(DEV_ADDR, vec![0x1F, 0b1100_0000]),
        // read CTRL_REG4
        I2cTrans::write_read(DEV_ADDR, vec![0x23], vec![0b0000_0000]),
        // write CTRL_REG4: BDU
        I2cTrans::write(DEV_ADDR, vec![0x23, 0b1000_0000]),
    ];
    let mock = I2cMock::new(&trans);
    block_on(async {
        let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).await.unwrap();
        dev.enable_temp(true).await.unwrap();
        dev.destroy().done();
    });
}

mod common;

use common::*;

#[test]
fn create() {
    let trans = [trans_who_am_i()];
    let mock = I2cMock::new(&trans);
    let dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();
    dev.destroy().done();
}

#[test]
fn create0() {
    let trans = [trans_who_am_i()];
    let mock = I2cMock::new(&trans);
    let dev = Lis2dh12::new(mock, SlaveAddr::Alternative(false)).unwrap();
    dev.destroy().done();
}

#[test]
fn create1() {
    let trans = [
        // WHO_AM_I on alternative slave address
        I2cTrans::write_read(DEV_ADDR | 0b1, vec![0x0F], vec![0b0011_0011]),
    ];
    let mock = I2cMock::new(&trans);
    let dev = Lis2dh12::new(mock, SlaveAddr::Alternative(true)).unwrap();
    dev.destroy().done();
}

#[test]
fn dev_id_get() {
    let trans = [trans_who_am_i(), trans_who_am_i()];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Alternative(false)).unwrap();
    let dev_id = dev.get_device_id().unwrap();
    assert_eq!(dev_id, 0b0011_0011);
    dev.destroy().done();
}

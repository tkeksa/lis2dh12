mod common;

use common::*;

#[test]
fn int1_disable() {
    let trans = [
        trans_who_am_i(),
        // write INT1_CFG: 0x00
        I2cTrans::write(DEV_ADDR, vec![0x30, 0x00]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();
    dev.int1().disable().unwrap();
    dev.destroy().done();
}

#[test]
fn int2_disable() {
    let trans = [
        trans_who_am_i(),
        // write INT2_CFG: 0x00
        I2cTrans::write(DEV_ADDR, vec![0x34, 0x00]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();
    dev.int2().disable().unwrap();
    dev.destroy().done();
}

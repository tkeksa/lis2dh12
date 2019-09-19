mod common;

use common::*;

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
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();
    dev.enable_temp(true).unwrap();
    dev.destroy().done();
}

#[test]
fn temp_disable() {
    let trans = [
        trans_who_am_i(),
        // read TEMP_CFG_REG
        I2cTrans::write_read(DEV_ADDR, vec![0x1F], vec![0b1111_1111]),
        // write TEMP_CFG_REG
        I2cTrans::write(DEV_ADDR, vec![0x1F, 0b0011_1111]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();
    dev.enable_temp(false).unwrap();
    dev.destroy().done();
}

#[test]
fn temp_status() {
    let trans = [
        trans_who_am_i(),
        // read STATUS_REG_AUX
        I2cTrans::write_read(DEV_ADDR, vec![0x07], vec![0b1011_1011]),
        I2cTrans::write_read(DEV_ADDR, vec![0x07], vec![0b0100_1011]),
        I2cTrans::write_read(DEV_ADDR, vec![0x07], vec![0b1011_0100]),
        I2cTrans::write_read(DEV_ADDR, vec![0x07], vec![0b0100_0100]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();

    let st = dev.get_temp_status().unwrap();
    assert_eq!(st, (false, false));

    let st = dev.get_temp_status().unwrap();
    assert_eq!(st, (true, false));

    let st = dev.get_temp_status().unwrap();
    assert_eq!(st, (false, true));

    let st = dev.get_temp_status().unwrap();
    assert_eq!(st, (true, true));

    dev.destroy().done();
}

#[test]
fn temp_out() {
    let trans = [
        trans_who_am_i(),
        // read OUT_TEMP_L, OUT_TEMP_L
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0x00]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0xFF, 0x7F]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0x80]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0xFF, 0xFF]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();

    let out = dev.get_temp_out().unwrap();
    assert_eq!(out, (0i8, 0u8));

    let out = dev.get_temp_out().unwrap();
    assert_eq!(out, (127i8, 255u8));

    let out = dev.get_temp_out().unwrap();
    assert_eq!(out, (-128i8, 0u8));

    let out = dev.get_temp_out().unwrap();
    assert_eq!(out, (-1i8, 255u8));

    dev.destroy().done();
}

#[test]
#[cfg(feature = "out_f32")]
fn temp_outf() {
    let trans = [
        trans_who_am_i(),
        // read OUT_TEMP_L, OUT_TEMP_L
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0x00]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x40, 0x00]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0xC0, 0xFF]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0x01]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0xFF]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0xC0, 0x7F]),
        I2cTrans::write_read(DEV_ADDR, vec![0x80 | 0x0C], vec![0x00, 0x80]),
    ];
    let mock = I2cMock::new(&trans);
    let mut dev = Lis2dh12::new(mock, SlaveAddr::Default).unwrap();

    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, 0f32);

    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, 0.25f32);
    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, -0.25f32);

    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, 1f32);
    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, -1f32);

    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, 127.75f32);
    let out = dev.get_temp_outf().unwrap();
    assert_eq!(out, -128f32);

    dev.destroy().done();
}

//! Platform-agnostic LIS2DH12 accelerometer driver which uses I2C via
//! [embedded-hal] and implements the [`Accelerometer` trait][trait]
//! from the `accelerometer` crate.
//!
//! Both a [`blocking`] and an [`asynch`] driver are provided, sharing the same
//! generated implementation via [bisync2]. The blocking API is also re-exported
//! at the crate root for convenience.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//!

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]

mod reg;

use core::fmt::Debug;

#[cfg(feature = "out_f32")]
pub use accelerometer::vector::F32x3;
pub use accelerometer::vector::I16x3;
pub use accelerometer::{Accelerometer, Error, ErrorKind, RawAccelerometer};

pub use crate::reg::{Aoi6d, FifoMode, FullScale, Mode, Odr};

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
            SlaveAddr::Default => reg::I2C_SAD,
            SlaveAddr::Alternative(a0) => reg::I2C_SAD | a0 as u8,
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

/// Errors returned from `detect_i2c_addr`
#[derive(Debug)]
pub enum AddrDetectionError<I2cError: Debug> {
    /// Other I2C error trying to detect a device address.
    I2c(I2cError),
    /// Invalid device ID read from device.
    InvalidDeviceId,
}

impl<I2cError: Debug> From<I2cError> for AddrDetectionError<I2cError> {
    fn from(value: I2cError) -> Self {
        AddrDetectionError::I2c(value)
    }
}

/// Blocking driver, built on [embedded-hal]'s [`I2c`](embedded_hal::i2c::I2c) trait.
///
/// [embedded-hal]: https://docs.rs/embedded-hal
#[path = "."]
pub mod blocking {
    use bisync2::synchronous::*;
    pub use embedded_hal::i2c::I2c;
    // `blocking::inner` and `asynch::inner` are deliberately the same file, compiled
    // twice under different `bisync2` module scopes.
    #[allow(clippy::duplicate_mod)]
    mod inner;
    pub use inner::*;
}
pub use blocking::*;

/// Async driver, built on [embedded-hal-async]'s [`I2c`](embedded_hal_async::i2c::I2c) trait.
///
/// [embedded-hal-async]: https://docs.rs/embedded-hal-async
#[path = "."]
pub mod asynch {
    use bisync2::asynchronous::*;
    pub use embedded_hal_async::i2c::I2c;
    // `blocking::inner` and `asynch::inner` are deliberately the same file, compiled
    // twice under different `bisync2` module scopes.
    #[allow(clippy::duplicate_mod)]
    mod inner;
    pub use inner::*;
}

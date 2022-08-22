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
#![forbid(unsafe_code)]

mod reg;

#[cfg(feature = "sync")]
mod sync;
#[cfg(feature = "sync")]
pub use crate::sync::{Int, Lis2dh12};

#[cfg(feature = "async")]
pub mod asynch;

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You must enable at least one of the `sync` or `async` features.");

pub use accelerometer::vector::I16x3;
pub use accelerometer::{Accelerometer, Error, ErrorKind, RawAccelerometer};

use crate::reg::*;
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

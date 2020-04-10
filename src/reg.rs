#![allow(non_upper_case_globals)]

#[cfg(feature = "out_f32")]
use cast::f32;
#[cfg(feature = "out_f32")]
use num_derive::FromPrimitive;

/// I2C slave address
pub const I2C_SAD: u8 = 0b001_1000;

pub const I2C_SUB_MULTI: u8 = 0b1000_0000;

/// Operating mode
pub enum Mode {
    /// High-resolution mode (12-bit data output)
    HighResolution,
    /// Normal mode (10-bit data output)
    Normal,
    /// Low-power mode (8-bit data output)
    LowPower,
}

/// Register mapping
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    STATUS_REG_AUX = 0x07,
    OUT_TEMP_L = 0x0C,
    OUT_TEMP_H = 0x0D,
    WHO_AM_I = 0x0F,
    CTRL_REG0 = 0x1E,
    TEMP_CFG_REG = 0x1F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    CTRL_REG6 = 0x25,
    REFERENCE = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_THS = 0x32,
    INT1_DURATION = 0x33,
    INT2_CFG = 0x34,
    INT2_SRC = 0x35,
    INT2_THS = 0x36,
    INT2_DURATION = 0x37,
    CLICK_CFG = 0x38,
    CLICK_SRC = 0x39,
    CLICK_THS = 0x3A,
    TIME_LIMIT = 0x3B,
    TIME_LATENCY = 0x3C,
    TIME_WINDOW = 0x3D,
    ACT_THS = 0x3E,
    ACT_DUR = 0x3F,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}

// === STATUS_REG_AUX (07h) ===

pub const TOR: u8 = 0b0100_0000;
pub const TDA: u8 = 0b0000_0100;

// === WHO_AM_I (0Fh) ===

/// WHO_AM_I device identification register
pub const DEVICE_ID: u8 = 0b0011_0011;

// === TEMP_CFG_REG (1Fh) ===

pub const TEMP_EN: u8 = 0b1100_0000;

// === CTRL_REG1 (20h) ===

pub const ODR_MASK: u8 = 0b1111_0000;

/// Output Data Rate
#[derive(Copy, Clone)]
#[cfg_attr(feature = "out_f32", derive(FromPrimitive))]
pub enum Odr {
    /// Power-down mode
    PowerDown = 0b0000,
    /// 1 Hz
    Hz1 = 0b0001,
    /// 10 Hz
    Hz10 = 0b0010,
    /// 25 Hz
    Hz25 = 0b0011,
    /// 50 Hz
    Hz50 = 0b0100,
    /// 100 Hz
    Hz100 = 0b0101,
    /// 200 Hz
    Hz200 = 0b0110,
    /// 400 Hz
    Hz400 = 0b0111,
    /// Low-power mode (1.620 kHz)
    HighRate0 = 0b1000,
    /// High-resolution / Normal (1.344 kHz),
    /// Low-power (5.376 kHz)
    HighRate1 = 0b1001,
}

pub const LPen: u8 = 0b0000_1000;
pub const Zen: u8 = 0b0000_0100;
pub const Yen: u8 = 0b0000_0010;
pub const Xen: u8 = 0b0000_0001;

// === CTRL_REG3 (22h) ===

pub const I1_CLICK: u8 = 0b1000_0000;
pub const I1_IA1: u8 = 0b0100_0000;
pub const I1_IA2: u8 = 0b0010_0000;
pub const I1_ZYXDA: u8 = 0b0001_0000;
pub const I1_WTM: u8 = 0b0000_0100;
pub const I1_OVERRUN: u8 = 0b0000_0010;

// === CTRL_REG4 (23h) ===

pub const BDU: u8 = 0b1000_0000;

pub const FS_MASK: u8 = 0b0011_0000;

/// Full-scale selection
#[derive(Copy, Clone)]
pub enum FullScale {
    /// ±2 g
    G2 = 0b00,
    /// ±4 g
    G4 = 0b01,
    /// ±8 g
    G8 = 0b10,
    /// ±16 g
    G16 = 0b11,
}

impl FullScale {
    #[cfg(feature = "out_f32")]
    pub(crate) fn convert_out_i16tof32(self, val: i16) -> f32 {
        // g/digit for high-resolution mode (12-bit)
        let sens: f32 = match self {
            Self::G2 => 0.001,
            Self::G4 => 0.002,
            Self::G8 => 0.004,
            Self::G16 => 0.012,
        };
        // up to 12-bit data, left-justified
        f32(val >> 4) * sens
    }
    #[cfg(feature = "out_f32")]
    pub(crate) fn convert_ths_f32tou8(self, val: f32) -> u8 {
        // 1LSb = x g
        let lsb: f32 = match self {
            Self::G2 => 0.016,
            Self::G4 => 0.032,
            Self::G8 => 0.062,
            Self::G16 => 0.186,
        };
        let f = val / lsb; // .round(); can not be used with no_std for now
        if f < 0.0 {
            0
        } else if f > 127.0 {
            0x7F
        } else {
            f as u8
        }
    }
}

pub const HR: u8 = 0b0000_1000;

// === CTRL_REG5 (24h) ===

pub const BOOT: u8 = 0b1000_0000;
pub const FIFO_EN: u8 = 0b0100_0000;
pub const LIR_INT1: u8 = 0b0000_1000;
pub const D4D_INT1: u8 = 0b0000_0100;
pub const LIR_INT2: u8 = 0b0000_0010;
pub const D4D_INT2: u8 = 0b0000_0001;

// === CTRL_REG6 (25h) ===

pub const I2_CLICK: u8 = 0b1000_0000;
pub const I2_IA1: u8 = 0b0100_0000;
pub const I2_IA2: u8 = 0b0010_0000;
pub const I2_BOOT: u8 = 0b0001_0000;
pub const I2_ACT: u8 = 0b0000_1000;
pub const INT_POLARITY: u8 = 0b0000_0010;

// === STATUS_REG (27h) ===

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;

// === FIFO_CTRL_REG (2Eh) ===

pub const FM_MASK: u8 = 0b1100_0000;

/// FIFO mode selection
#[derive(Copy, Clone)]
pub enum FifoMode {
    /// Bypass mode
    Bypass = 0b00,
    /// FIFO mode
    Fifo = 0b01,
    /// Stream mode
    Stream = 0b10,
    /// Stream-to-FIFO mode
    StreamToFifo = 0b11,
}

pub const FTH_MASK: u8 = 0b0001_1111;

// === INT1_CFG (30h), INT2_CFG (34h) ===

pub const AOI_6D_MASK: u8 = 0b1100_0000;

/// AOI-6D Interrupt mode
#[derive(Copy, Clone)]
pub enum Aoi6d {
    /// OR combination of interrupt events
    Or = 0b00,
    /// 6-direction movement recognition
    Movement6D = 0b01,
    /// AND combination of interrupt events
    And = 0b10,
    /// 6-direction position recognition
    Position6D = 0b11,
}

pub const ZHIE: u8 = 0b0010_0000;
pub const ZLIE: u8 = 0b0001_0000;
pub const YHIE: u8 = 0b0000_1000;
pub const YLIE: u8 = 0b0000_0100;
pub const XHIE: u8 = 0b0000_0010;
pub const XLIE: u8 = 0b0000_0001;

// === INT1_SRC (31h), INT2_SRC (35h) ===

pub const IA: u8 = 0b0100_0000;
pub const ZH: u8 = 0b0010_0000;
pub const ZL: u8 = 0b0001_0000;
pub const YH: u8 = 0b0000_1000;
pub const YL: u8 = 0b0000_0100;
pub const XH: u8 = 0b0000_0010;
pub const XL: u8 = 0b0000_0001;

// === INT1_THS (32h), INT2_THS (36h) ===

pub const THS_MASK: u8 = 0b0111_1111;

// === INT1_DURATION (33h), INT2_DURATION (37h) ===

pub const D_MASK: u8 = 0b0111_1111;

// === CLICK_CFG (38h) ===

pub const ZD: u8 = 0b0010_0000;
pub const ZS: u8 = 0b0001_0000;
pub const YD: u8 = 0b0000_1000;
pub const YS: u8 = 0b0000_0100;
pub const XD: u8 = 0b0000_0010;
pub const XS: u8 = 0b0000_0001;

// === CLICK_SRC (39h) ===

pub const DClick: u8 = 0b0010_0000;
pub const SClick: u8 = 0b0001_0000;
pub const Sign: u8 = 0b0000_1000;
pub const Z: u8 = 0b0000_0100;
pub const Y: u8 = 0b0000_0010;
pub const X: u8 = 0b0000_0001;

// === CLICK_THS (3Ah) ===

pub const LIR_Click: u8 = 0b1000_0000;

// === TIME_LIMIT (3Bh) ===

pub const TLI_MASK: u8 = 0b0111_1111;

// === ACT_THS (3Eh) ===

pub const Acth_MASK: u8 = 0b0111_1111;

// *** INT registers ***

pub trait IntRegs {
    fn reg_cfg() -> Register;
    fn reg_src() -> Register;
    fn reg_ths() -> Register;
    fn reg_duration() -> Register;
}

macro_rules! int_regs {
    ($INTX:ident: ($REG_CFG:ident, $REG_SRC:ident, $REG_THS:ident, $REG_DURATION:ident)) => {
        pub struct $INTX();
        impl IntRegs for $INTX {
            fn reg_cfg() -> Register {
                Register::$REG_CFG
            }
            fn reg_src() -> Register {
                Register::$REG_SRC
            }
            fn reg_ths() -> Register {
                Register::$REG_THS
            }
            fn reg_duration() -> Register {
                Register::$REG_DURATION
            }
        }
    };
}

int_regs!(Int1Regs: (INT1_CFG, INT1_SRC, INT1_THS, INT1_DURATION));
int_regs!(Int2Regs: (INT2_CFG, INT2_SRC, INT2_THS, INT2_DURATION));

#[cfg(any(feature = "tracing", feature = "defmt"))]
use defmt::Format;

pub(crate) const DEFAULT_CONFIG_MSG: &[u8] = &[
    0x00, // first byte of register to write to
    0x2d, // second byte of register to write to
    // value    addr : description
    0x12, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
    0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x11, // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1)
    0x02, // 0x31 : bit 1 = interrupt depending on the polarity
    0x00, // 0x32 : not user-modifiable
    0x02, // 0x33 : not user-modifiable
    0x08, // 0x34 : not user-modifiable
    0x00, // 0x35 : not user-modifiable
    0x08, // 0x36 : not user-modifiable
    0x10, // 0x37 : not user-modifiable
    0x01, // 0x38 : not user-modifiable
    0x01, // 0x39 : not user-modifiable
    0x00, // 0x3a : not user-modifiable
    0x00, // 0x3b : not user-modifiable
    0x00, // 0x3c : not user-modifiable
    0x00, // 0x3d : not user-modifiable
    0xFF, // 0x3e : not user-modifiable
    0x00, // 0x3f : not user-modifiable
    0x0F, // 0x40 : not user-modifiable
    0x00, // 0x41 : not user-modifiable
    0x00, // 0x42 : not user-modifiable
    0x00, // 0x43 : not user-modifiable
    0x00, // 0x44 : not user-modifiable
    0x00, // 0x45 : not user-modifiable
    0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
    0x0B, // 0x47 : not user-modifiable
    0x00, // 0x48 : not user-modifiable
    0x00, // 0x49 : not user-modifiable
    0x02, // 0x4a : not user-modifiable
    0x14, // 0x4b : not user-modifiable
    0x21, // 0x4c : not user-modifiable
    0x00, // 0x4d : not user-modifiable
    0x00, // 0x4e : not user-modifiable
    0x05, // 0x4f : not user-modifiable
    0x00, // 0x50 : not user-modifiable
    0x00, // 0x51 : not user-modifiable
    0x00, // 0x52 : not user-modifiable
    0x00, // 0x53 : not user-modifiable
    0xC8, // 0x54 : not user-modifiable
    0x00, // 0x55 : not user-modifiable
    0x00, // 0x56 : not user-modifiable
    0x38, // 0x57 : not user-modifiable
    0xFF, // 0x58 : not user-modifiable
    0x01, // 0x59 : not user-modifiable
    0x00, // 0x5a : not user-modifiable
    0x08, // 0x5b : not user-modifiable
    0x00, // 0x5c : not user-modifiable
    0x00, // 0x5d : not user-modifiable
    0x01, // 0x5e : not user-modifiable
    0xCC, // 0x5f : not user-modifiable
    0x07, // 0x60 : not user-modifiable
    0x01, // 0x61 : not user-modifiable
    0xF1, // 0x62 : not user-modifiable
    0x05, // 0x63 : not user-modifiable
    0x00, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
    0xA0, // 0x65 : Sigma threshold LSB
    0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
    0x80, // 0x67 : Min count Rate LSB
    0x08, // 0x68 : not user-modifiable
    0x38, // 0x69 : not user-modifiable
    0x00, // 0x6a : not user-modifiable
    0x00, // 0x6b : not user-modifiable
    0x00, // 0x6c : Intermeasurement period MSB, 32 bits register
    0x00, // 0x6d : Intermeasurement period
    0x0F, // 0x6e : Intermeasurement period
    0x89, // 0x6f : Intermeasurement period LSB
    0x00, // 0x70 : not user-modifiable
    0x00, // 0x71 : not user-modifiable
    0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB)
    0x00, // 0x73 : distance threshold high LSB
    0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
    0x00, // 0x75 : distance threshold low LSB
    0x00, // 0x76 : not user-modifiable
    0x01, // 0x77 : not user-modifiable
    0x07, // 0x78 : not user-modifiable
    0x05, // 0x79 : not user-modifiable
    0x06, // 0x7a : not user-modifiable
    0x06, // 0x7b : not user-modifiable
    0x00, // 0x7c : not user-modifiable
    0x00, // 0x7d : not user-modifiable
    0x02, // 0x7e : not user-modifiable
    0xC7, // 0x7f : not user-modifiable
    0xFF, // 0x80 : not user-modifiable
    0x9B, // 0x81 : not user-modifiable
    0x00, // 0x82 : not user-modifiable
    0x00, // 0x83 : not user-modifiable
    0x00, // 0x84 : not user-modifiable
    0x01, // 0x85 : not user-modifiable
    0x00, // 0x86 : clear interrupt, 0x01=clear
    0x00, // 0x87 : ranging, 0x00=stop, 0x40=start
];

/// A register on the device, identified by a 16-bit address.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(any(feature = "defmt", feature = "tracing"), derive(Format))]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
pub enum Register {
    I2C_SLAVE_DEVICE_ADDRESS = 0x0001,
    OSC_FREQ = 0x0006,
    VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
    /// This name is temporary.
    MYSTERY_1 = 0x000b,
    /// This name is also temporary.
    MYSTERY_2 = 0x0024,
    SYSTEM_START = 0x0087,
    GPIO_HV_MUX_CTRL = 0x0030,
    GPIO_TIO_HV_STATUS = 0x0031,
    RANGE_CONFIG_A = 0x005e,
    RANGE_CONFIG_B = 0x0061,
    INTERMEASUREMENT_MS = 0x006c,
    SYSTEM_INTERRUPT_CLEAR = 0x0086,
    RESULT_RANGE_STATUS = 0x0089,
    RESULT_NUM_SPADS = 0x008c,
    RESULT_SIGNAL_RATE = 0x008e,
    RESULT_AMBIENT_RATE = 0x0090,
    RESULT_SIGMA = 0x0092,
    RESULT_DISTANCE = 0x0096,
    RESULT_OSC_CALIBRATE_VAL = 0x00de,
    SYSTEM_STATUS = 0x00e5,
    IDENTIFICATION_MODEL_ID = 0x010f,
}

impl Register {
    /// Get the 16-bit address of the register.
    ///
    /// ```
    /// # use vl53l4cd::Register;
    /// assert_eq!(0x010f, Register::IDENTIFICATION_MODEL_ID.addr());
    /// ```
    pub const fn addr(&self) -> u16 {
        *self as u16
    }

    /// Get the big-endian bytes of the register address.
    pub const fn as_bytes(&self) -> [u8; 2] {
        self.addr().to_be_bytes()
    }
}

/// Default I²C address of the VL53L4CD.
pub const DEFAULT_SLAVE_ADDR: u16 = 0x29;

/// Measurement status as per the user manual.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
#[cfg_attr(any(feature = "defmt", feature = "tracing"), derive(Format))]
pub enum Status {
    /// Returned distance is valid.
    Valid = 0,
    /// Sigma is above the defined threshol.
    SigmaAboveThreshold,
    /// Signal is below the defined threshold.
    SigmaBelowThreshold,
    /// Measured distance is below detection threshold.
    DistanceBelowDetectionThreshold,
    /// Phase out of valid limit.
    InvalidPhase,
    /// Hardware failure.
    HardwareFail,
    /// Phase valid but no wrap around check performed.
    NoWrapAroundCheck,
    /// Wrapped target, phase does not match.
    WrappedTargetPhaseMismatch,
    /// Processing fail.
    ProcessingFail,
    /// Crosstalk signal fail.
    XTalkFail,
    /// Interrupt error.
    InterruptError,
    /// Merged target.
    MergedTarget,
    /// Too low signal.
    SignalTooWeak,
    /// Other error (e.g. boot error).
    Other = 255,
}

/// Severity of a measurement status.
#[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord)]
pub enum Severity {
    /// The measurement is completely valid.
    None,
    /// The computed measurement might be somewhat correct.
    Warning,
    /// Something went very wrong.
    Error,
}

impl Status {
    pub const fn from_rtn(rtn: u8) -> Self {
        assert!(rtn < 24); // if rtn >= 24, return rtn

        match rtn {
            3 => Self::HardwareFail,
            4 | 5 => Self::SigmaBelowThreshold,
            6 => Self::SigmaAboveThreshold,
            7 => Self::WrappedTargetPhaseMismatch,
            8 => Self::DistanceBelowDetectionThreshold,
            9 => Self::Valid,
            12 => Self::XTalkFail,
            13 => Self::InterruptError,
            18 => Self::InterruptError,
            19 => Self::NoWrapAroundCheck,
            22 => Self::MergedTarget,
            23 => Self::SignalTooWeak,
            _ => Self::Other,
        }
    }

    /// Severity of this status as per the user manual.
    pub const fn severity(&self) -> Severity {
        match self {
            Status::Valid => Severity::None,
            Status::SigmaAboveThreshold => Severity::Warning,
            Status::SigmaBelowThreshold => Severity::Warning,
            Status::DistanceBelowDetectionThreshold => Severity::Error,
            Status::InvalidPhase => Severity::Error,
            Status::HardwareFail => Severity::Error,
            Status::NoWrapAroundCheck => Severity::Warning,
            Status::WrappedTargetPhaseMismatch => Severity::Error,
            Status::ProcessingFail => Severity::Error,
            Status::XTalkFail => Severity::Error,
            Status::InterruptError => Severity::Error,
            Status::MergedTarget => Severity::Error,
            Status::SignalTooWeak => Severity::Error,
            Status::Other => Severity::Error,
        }
    }
}
//! Async driver for the [VL53L4CD ToF distance sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html).
//!
//! ```no_run
#![doc = include_str!("../examples/linux.rs")]
//! ```

#![warn(missing_docs)]
#![no_std]

use core::fmt::Debug;

#[cfg(feature = "tracing")]
use tracing::{debug, error, instrument};

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
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
pub enum Register {
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
    const fn from_rtn(rtn: u8) -> Self {
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

/// A VL53L4CD measurement.
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// Validity of the measurement.
    pub status: Status,
    /// Measured distance to the target (millimeters).
    pub distance: u16,
    /// Ambient rate measurement performed on the
    /// return array, with no active photon emission, to
    /// measure the ambient signal rate due to noise.
    ///
    /// The returned value is specified in thousand counts
    /// per second (kcps).
    pub ambient_rate: u16,
    /// Number of detected photos during the VCSEL pulse.
    ///
    /// The returned value is specified in thousand counts
    /// per second (kcps).
    pub signal_rate: u16,
    /// Number of SPADs enabled for this measurement. Targets that
    /// are far away or have low reflectance will activate
    /// more SPADs.
    pub spads_enabled: u16,
    /// Sigma estimator for the noise in the reported
    /// target distance (millimeters).
    pub sigma: u16,
}

impl Measurement {
    /// Whether this measurement is valid or not, given its status.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.status == Status::Valid
    }
}

/// A VL53L4CD ToF range sensor.
pub struct Vl53l4cd<I> {
    i2c: I,
    slave_addr: u8,
}

impl<I, E> Vl53l4cd<I>
where
    I: embedded_hal_async::i2c::I2c<Error = E>,
{
    /// Construct a new sensor, without sending
    /// any commands. To begin measuring, you
    /// need to call [`Self::init`] as well as
    /// [`Self::start_ranging`].
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR as u8,
        }
    }

    /// Same as `new`, but the chip address can be specified given that it should be adjustable
    pub fn new_with_addr(i2c: I, addr: u8) -> Self {
        Self {
            i2c,
            slave_addr: addr,
        }
    }

    /// Initialize the sensor.
    ///
    /// # Errors
    ///
    /// If the device id reported by the sensor isn't `0xebaa`, this
    /// function returns an error. This is mostly done to prevent
    /// strange I²C bugs where all returned bytes are zeroed.
    #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn init(&mut self) -> Result<(), Vl53l4cdError<E>> {
        let id = self.read_word(Register::IDENTIFICATION_MODEL_ID).await?;
        if id != 0xebaa {
            #[cfg(feature = "tracing")]
            error!("strange device id `{:#06x}`", id);
            return Err(Vl53l4cdError::InvalidArgument);
        }

        #[cfg(feature = "tracing")]
        debug!("waiting for boot");

        while self.read_byte(Register::SYSTEM_STATUS).await? != 0x3 {
            #[cfg(feature = "tokio")] // TODO: allow the user to implement sleep
            tokio::time::sleep(core::time::Duration::from_millis(1)).await; // wait for boot
        }

        #[cfg(feature = "tracing")]
        debug!("booted");

        self.i2c.write(self.slave_addr, DEFAULT_CONFIG_MSG).await?;

        // start VHV
        self.start_ranging().await?;
        self.stop_ranging().await?;
        self.write_byte(
            Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND,
            0x09,
        )
        .await?;
        self.write_byte(Register::MYSTERY_1, 0).await?;
        self.write_word(Register::MYSTERY_2, 0x500).await?;

        self.set_range_timing(50, 0).await?;

        Ok(())
    }

    /// Set the range timing for this sensor. The timing budget *must*
    /// be greater than or equal to 10 ms and less than or equal to 200 ms.
    /// From the manufacturer's user manual:
    ///
    /// > The range timing is a single function which allows the user to define the VCSEL timeout and the ranging
    /// > frequency of the sensor. It is composed of two elements:
    /// > * The Timing budget: It corresponds to the VCSEL enabled time. The user can choose a value between 10 ms
    /// > and 200 ms. If the InterMeasurement is set to 0, the VCSEL is always enabled, so the TimingBudget is
    /// > equal to the ranging period between consecutive measurements.
    /// > * The InterMeasurement: It allows the user to define the time between two consecutive measurements.
    /// > To use the InterMeasurement, the user needs to set a value greater than the TimingBudget. When the
    /// > TimingBudget is consumed, the device goes into low power mode until the InterMeasurement is reached. A
    /// > value set to 0 disables the InterMeasurement.
    ///
    /// # Panics
    ///
    /// Panics if the timing budget is less than 10 ms or more than 200 ms,
    /// or if the timing budget is less than the inter-measurement time
    /// (except when the inter-measurement time is zero).
    ///
    /// If the oscillation frequency reported by the sensor (2 bytes starting
    /// at [`OSC_FREQ`]) is zero, this function panics.
    ///
    /// [`OSC_FREQ`]: Register#variant.OSC_FREQ
    #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn set_range_timing(
        &mut self,
        timing_budget_ms: u32,
        inter_measurement_ms: u32,
    ) -> Result<(), Vl53l4cdError<E>> {
        assert!(
            (10..=200).contains(&timing_budget_ms),
            "timing budget must be in range [10, 200]"
        );

        let osc_freq = self.read_word(Register::OSC_FREQ).await?;

        if osc_freq == 0 {
            #[cfg(feature = "tracing")]
            error!("oscillation frequency is zero");
            return Err(Vl53l4cdError::InvalidArgument);
        }

        let mut timing_budget_us = timing_budget_ms * 1000;

        if inter_measurement_ms == 0 {
            // continuous mode
            self.write_dword(Register::INTERMEASUREMENT_MS, 0).await?;
            timing_budget_us -= 2500;
        } else {
            assert!(
                inter_measurement_ms <= timing_budget_ms,
                "timing budget must be greater than or equal to inter-measurement"
            );

            // autonomous low power mode
            let clock_pll = u32::from(
                self.read_word(Register::RESULT_OSC_CALIBRATE_VAL).await? & 0x3ff,
            );
            let inter_measurement_fac = 1.055 * (inter_measurement_ms * clock_pll) as f32;
            self.write_dword(
                Register::INTERMEASUREMENT_MS,
                inter_measurement_fac as u32,
            )
            .await?;

            timing_budget_us -= 4300;
            timing_budget_us /= 2;
        }

        let (a, b) = range_config_values(timing_budget_us, osc_freq);

        self.write_word(Register::RANGE_CONFIG_A, a).await?;
        self.write_word(Register::RANGE_CONFIG_B, b).await?;

        Ok(())
    }

    /// Wait for a measurement to be available on the sensor and then read
    /// the measurement. This function polls the sensor for a measurement
    /// until one is available, reads the measurement and finally clears
    /// the interrupt in order to request another measurement.
    #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn measure(&mut self) -> Result<Measurement, Vl53l4cdError<E>> {
        self.wait_for_measurement().await?;

        #[cfg(feature = "tracing")]
        debug!("measurement ready; reading");

        let measurement = self.read_measurement().await?;
        self.clear_interrupt().await?;

        Ok(measurement)
    }

    /// Adjust the sensor to prevent the measurements from deviating due to
    /// ambient temperature variations. The ranging needs to be stopped with
    /// [`Self::stop_ranging`] before calling this function.
    ///
    /// > Ambient temperature has an effect on ranging accuracy. In order to ensure the best performances, a temperature
    /// > update needs to be applied to the sensor. This update needs to be performed when the temperature might have
    /// > changed by more than 8 degrees Celsius.
    #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn start_temperature_update(&mut self) -> Result<(), Vl53l4cdError<E>> {
        self.write_byte(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81)
            .await?;
        self.write_byte(Register::MYSTERY_1, 0x92).await?;
        self.write_byte(Register::SYSTEM_START, 0x40).await?;

        self.wait_for_measurement().await?;
        self.clear_interrupt().await?;
        self.stop_ranging().await?;

        self.write_byte(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
            .await?;
        self.write_byte(Register::MYSTERY_1, 0).await?;

        Ok(())
    }

    /// Poll the sensor until a measurement is ready.
    #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn wait_for_measurement(&mut self) -> Result<(), Vl53l4cdError<E>> {
        #[cfg(feature = "tracing")]
        debug!("waiting for measurement");

        #[cfg(feature = "tokio")]
        match tokio::time::timeout(core::time::Duration::from_secs(1), async move {
            while !self.has_measurement().await? {
                tokio::time::sleep(core::time::Duration::from_millis(1)).await;
            }
            Ok(())
        })
        .await
        {
            Ok(res) => res,
            Err(_) => {
                #[cfg(feature = "tracing")]
                error!("timeout waiting for measurement");
                Err(Vl53l4cdError::Timeout)
            }
        }

        #[cfg(not(feature = "tokio"))]
        {
            while !self.has_measurement().await? {}
            Ok(())
        }
    }

    /// Check if the sensor has a measurement ready. Unless you really like
    /// low-level, use the more ergonomic [`Self::measure`] instead.
    #[inline]
    pub async fn has_measurement(&mut self) -> Result<bool, Vl53l4cdError<E>> {
        let ctrl = self.read_byte(Register::GPIO_HV_MUX_CTRL).await?;
        let status = self.read_byte(Register::GPIO_TIO_HV_STATUS).await?;
        Ok(status & 1 != ctrl >> 4 & 1)
    }

    /// Read the current measurement from the sensor. Wait for
    /// [`Self::has_measurement`] to return true before running this so that
    /// the measurement doesn't get overwritten halfway through you reading it.
    /// Instruct the sensor to resume measuring with [`Self::clear_interrupt`]
    /// afterwards.
    ///
    /// ```no_run
    /// # use vl53l4cd::{Vl53l4cd, i2c};
    /// #
    /// # tokio_test::block_on(async {
    /// # let mut vl53 = Vl53l4cd::new(i2c::Mock::default());
    /// loop {
    ///     while !vl53.has_measurement().await? { }
    ///
    ///     let measurement = vl53.read_measurement().await?;
    ///     vl53.clear_interrupt().await?;
    ///
    ///     println!("{} mm", measurement.distance);
    /// }
    /// # Ok::<(), vl53l4cd::Error<<i2c::Mock as i2c::Device>::Error>>(())
    /// # });
    /// ```
    #[inline]
    #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn read_measurement(&mut self) -> Result<Measurement, Vl53l4cdError<E>> {
        let status = self.read_byte(Register::RESULT_RANGE_STATUS).await? & 0x1f;

        Ok(Measurement {
            status: Status::from_rtn(status),
            distance: self.read_word(Register::RESULT_DISTANCE).await?,
            spads_enabled: self.read_word(Register::RESULT_NUM_SPADS).await? / 256,
            ambient_rate: self.read_word(Register::RESULT_AMBIENT_RATE).await? * 8,
            signal_rate: self.read_word(Register::RESULT_SIGNAL_RATE).await? * 8,
            sigma: self.read_word(Register::RESULT_SIGMA).await? / 4,
        })
    }

    /// Clear the interrupt which will eventually trigger a new measurement.
    #[inline]
    #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn clear_interrupt(&mut self) -> Result<(), Vl53l4cdError<E>> {
        self.write_byte(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)
            .await?;
        Ok(())
    }

    /// Begin ranging.
    #[inline]
    #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn start_ranging(&mut self) -> Result<(), Vl53l4cdError<E>> {
        if self.read_word(Register::INTERMEASUREMENT_MS).await? == 0 {
            // autonomous mode
            self.write_byte(Register::SYSTEM_START, 0x21).await?;
        } else {
            // continuous mode
            self.write_byte(Register::SYSTEM_START, 0x40).await?;
        }

        self.wait_for_measurement().await?;
        self.clear_interrupt().await
    }

    /// Stop ranging.
    #[inline]
    #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn stop_ranging(&mut self) -> Result<(), Vl53l4cdError<E>> {
        self.write_byte(Register::SYSTEM_START, 0x00).await?;
        Ok(())
    }

    /// Writes byte to register
    pub async fn write_byte(&mut self, reg: Register, byte: u8) -> Result<(), Vl53l4cdError<E>> {
        let mut msg = [0, 0, byte];
        msg[..2].copy_from_slice(&reg.as_bytes());

        self.i2c
            .write(self.slave_addr, &msg)
            .await
            .map_err(Vl53l4cdError::I2c)?;
        Ok(())
    }

    /// Reads byte from register
    pub async fn read_byte(&mut self, reg: Register) -> Result<u8, Vl53l4cdError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.slave_addr, &reg.as_bytes(), &mut byte)
            .await
            .map_err(Vl53l4cdError::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub async fn read_bytes(
        &mut self,
        reg: Register,
        buf: &mut [u8],
    ) -> Result<(), Vl53l4cdError<E>> {
        self.i2c
            .write_read(self.slave_addr, &reg.as_bytes(), buf)
            .await
            .map_err(Vl53l4cdError::I2c)?;
        Ok(())
    }

    /// IDK WHAT THE DIFFERENCE BETWEEN A SET OF BYTES AND A WORD
    /// This seems to just be an abstraction of read_bytes that returns a u16 instead of filling a buffer
    async fn read_word(&mut self, reg: Register) -> Result<u16, Vl53l4cdError<E>> {
        let mut buf = [0; 2];
        self.read_bytes(reg, &mut buf).await?;
        Ok(u16::from_be_bytes(buf))
    }

    async fn write_word(&mut self, reg: Register, data: u16) -> Result<(), Vl53l4cdError<E>> {
        let mut msg = [0; 4];
        msg[..2].copy_from_slice(&reg.as_bytes());
        msg[2..].copy_from_slice(&data.to_be_bytes());
        self.i2c.write(self.slave_addr, &msg).await?;
        Ok(())
    }

    async fn write_dword(&mut self, reg: Register, data: u32) -> Result<(), Vl53l4cdError<E>> {
        let mut msg = [0; 6];
        msg[..2].copy_from_slice(&reg.as_bytes());
        msg[2..].copy_from_slice(&data.to_be_bytes());
        self.i2c.write(self.slave_addr, &msg).await?;
        Ok(())
    }
}

/// Calculate valid values for [`Register::RANGE_CONFIG_A`] and
/// [`Register::RANGE_CONFIG_B`].
///
/// ```
/// let (a, b) = vl53l4cd::range_config_values(197500, 48250);
///
/// assert_eq!(a, 0x04fc);
/// assert_eq!(b, 0x05a8);
/// ```
pub fn range_config_values(mut timing_budget_us: u32, osc_freq: u16) -> (u16, u16) {
    // I didn't make these values up because I'm not a wizard.
    // https://github.com/stm32duino/VL53L4CD/blob/b64ff4fa877c3cf156e11639e5fa305208dd3be9/src/vl53l4cd_api.cpp#L370

    let macro_period_us = (2304 * (0x40000000 / u32::from(osc_freq))) >> 6;
    timing_budget_us <<= 12;

    let f = |x: u32| {
        let mut ms_byte = 0;
        let tmp = macro_period_us * x;
        let mut ls_byte = ((timing_budget_us + (tmp >> 7)) / (tmp >> 6)) - 1;

        while (ls_byte & 0xffffff00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }

        (ms_byte << 8) | (ls_byte & 0xff) as u16
    };

    (f(16), f(12))
}

/// VL53L4CD driver error. In order to get more details,
/// make sure that the `tracing` feature is enabled.
#[derive(Debug)]
pub enum Vl53l4cdError<E> {
    /// I²C (I/O) error.
    I2c(E),

    /// Invalid argument, often as a result of an I/O
    /// error.
    InvalidArgument,

    /// Timeout waiting for the sensor.
    Timeout,
}

impl<E> From<E> for Vl53l4cdError<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}

//! Async driver for the [VL53L4CD ToF distance sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html).

#![warn(missing_docs)]
#![no_std]

mod device;

use device::*;

use embedded_hal_async::delay::DelayUs;
#[cfg(feature = "tracing")]
use defmt::{debug, error, Format};
#[cfg(feature = "defmt")]
use defmt::Format;

/// A VL53L4CD measurement.
#[derive(Debug)]
#[cfg_attr(any(feature = "defmt", feature = "tracing"), derive(Format))]
pub struct Measurement {
    /// Validity of the measurement.
    pub status: device::Status,
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
    pub async fn init<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Vl53l4cdError<E>> {
        let id = self.read_word(Register::IDENTIFICATION_MODEL_ID).await?;
        if id != 0xebaa {
            #[cfg(feature = "tracing")]
            error!("strange device id `{:#06x}`", id);
            return Err(Vl53l4cdError::InvalidArgument);
        }

        #[cfg(feature = "tracing")]
        debug!("waiting for boot");

        while self.read_byte(Register::SYSTEM_STATUS).await? != 0x3 {
            delay.delay_ms(1).await; // wait for boot
        }

        #[cfg(feature = "tracing")]
        debug!("booted");

        self.i2c.write(self.slave_addr, DEFAULT_CONFIG_MSG).await?;

        // start VHV
        self.start_ranging(delay).await?;
        self.stop_ranging().await?;
        self.write_byte(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
            .await?;
        self.write_byte(Register::MYSTERY_1, 0).await?;
        self.write_word(Register::MYSTERY_2, 0x500).await?;

        self.set_range_timing(50, 0).await?;

        Ok(())
    }

    /// This function sets a new I2C address to a sensor. It can be used when multiple sensors share the
    /// same I2C bus.
    /// 
    /// **It appears that the ToF sensors do not remember their reassigned I2C address. They must be individually
    /// reassigned on boot every time the power is cycled. This can be done by pulling the XSHUT pin low on all but
    /// one sensor.**
    /// 
    /// Returns Ok() if successful
    pub async fn set_i2c_addr(&mut self, new_addr: u8) -> Result<(), Vl53l4cdError<E>> {
        let res = self.write_byte(Register::I2C_SLAVE_DEVICE_ADDRESS, new_addr).await;
        if res.is_ok() {
            self.slave_addr = new_addr;
            return Ok(());
        }
        return res
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
            let clock_pll =
                u32::from(self.read_word(Register::RESULT_OSC_CALIBRATE_VAL).await? & 0x3ff);
            let inter_measurement_fac = 1.055 * (inter_measurement_ms * clock_pll) as f32;
            self.write_dword(Register::INTERMEASUREMENT_MS, inter_measurement_fac as u32)
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
    // #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn measure<D: DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Vl53l4cdError<E>> {
        let m = self.wait_for_measurement(delay).await;

        if m.is_err() {
            self.clear_interrupt().await?;
            return Err(m.unwrap_err());
        }

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
    // #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn start_temperature_update<D: DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Vl53l4cdError<E>> {
        self.write_byte(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81)
            .await?;
        self.write_byte(Register::MYSTERY_1, 0x92).await?;
        self.write_byte(Register::SYSTEM_START, 0x40).await?;

        self.wait_for_measurement(delay).await?;
        self.clear_interrupt().await?;
        self.stop_ranging().await?;

        self.write_byte(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
            .await?;
        self.write_byte(Register::MYSTERY_1, 0).await?;

        Ok(())
    }

    /// Poll the sensor until a measurement is ready.
    // #[cfg_attr(feature = "tracing", instrument(skip(self)))]
    pub async fn wait_for_measurement<D: DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Vl53l4cdError<E>> {
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
            let mut waited: u16 = 0;
            while !self.has_measurement().await? {
                if waited > 5 {
                    #[cfg(feature = "tracing")]
                    error!("timeout waiting for measurement");
                    return Err(Vl53l4cdError::Timeout);
                }
                delay.delay_ms(1).await;
                waited += 1;
            }
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
    pub async fn clear_interrupt(&mut self) -> Result<(), Vl53l4cdError<E>> {
        self.write_byte(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)
            .await?;
        Ok(())
    }

    /// Begin ranging.
    // #[inline]
    // #[cfg_attr(feature = "tracing", instrument(level = "debug", skip(self)))]
    pub async fn start_ranging<D: DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Vl53l4cdError<E>> {
        if self.read_word(Register::INTERMEASUREMENT_MS).await? == 0 {
            // autonomous mode
            self.write_byte(Register::SYSTEM_START, 0x21).await?;
        } else {
            // continuous mode
            self.write_byte(Register::SYSTEM_START, 0x40).await?;
        }

        self.wait_for_measurement(delay).await?;
        self.clear_interrupt().await
    }

    /// Stop ranging.
    #[inline]
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
#[cfg_attr(any(feature = "defmt", feature = "tracing"), derive(Format))]
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

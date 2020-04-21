//! An embedded-hal driver for the PZEM004T energy monitor.
//!
//! The driver must be initialized by passing a Serial interface peripheral
//! to the [`Pzem::new`](struct.Pzem.html#method.new)
//! function, which in turn will create a driver instance, with the slave address specified,
//! or the default general address for a single-slave environment `0xf8`.
//!
//! # Examples
//!
//! Examples can be found in the [`examples/`](https://github.com/iostapyshyn/pzem004t/tree/master/examples) directory.
//!
//! ## Read the measurements off the sensor every second
//!
//!     let mut pzem = pzem004t::Pzem::new(serial, None).unwrap();
//!     let mut m = pzem004t::Measurement::default();
//!     loop {
//!         match pzem.read(&mut m, Some((&mut tim, TIMEOUT))) {
//!             Err(e) => println!("Could not read PZEM004T: {}", e);
//!             Ok(()) => {
//!                 println!("Voltage: {:.1} V", m.voltage);
//!                 println!("Current: {:.3} A", m.current);
//!                 println!("Power: {:.1} W", m.power);
//!                 println!("Energy: {:.3} kWh", m.energy);
//!                 println!("Frequency: {:.1} Hz", m.frequency);
//!                 println!("Power factor: {:.2}", m.pf);
//!                 println!("Alarm: {}\n", m.alarm);
//!             }
//!         }
//!
//!         tim.start(1.hz());
//!         block!(tim.wait()).unwrap();
//!     }

#![no_std]

extern crate crc16;
extern crate embedded_hal as hal;

#[macro_use(block)]
extern crate nb;

mod io;
use io::*;

mod no_timeout;
pub use no_timeout::NoTimeout;

use core::fmt::Display;
use core::fmt::Formatter;
use hal::serial;
use hal::timer;

const ADDR_DEFAULT: u8 = 0xf8; // Universal address for single-slave environment
const ADDR_MIN: u8 = 0x01;
const ADDR_MAX: u8 = 0xf7;

const CMD_READ: u8 = 0x04; // Read the measurement registers
const CMD_RESET: u8 = 0x42; // Reset the energy counter

const CMD_READ_PARAM: u8 = 0x03; // Read the slave parameters
const CMD_WRITE_PARAM: u8 = 0x06; // Write the slave parameters

const PARAM_THRESHOLD: u16 = 0x0001; // Power alarm threshold
const PARAM_ADDR: u16 = 0x0002; // Modbus-RTU address

const REG_COUNT: u16 = 10; // 10 registers in total

/// Errors which can occur when attempting to communicate with PZEM004T sensor.
#[derive(Debug, Clone)]
pub enum Error<WriteError, ReadError> {
    TimedOut,
    CrcMismatch,
    PzemError,
    IllegalAddress,
    WriteError(WriteError),
    ReadError(ReadError),
}

impl<WriteError: Display, ReadError: Display> Display for Error<WriteError, ReadError> {
    fn fmt(&self, f: &mut Formatter) -> Result<(), core::fmt::Error> {
        match self {
            Error::TimedOut => write!(f, "Communication timed out"),
            Error::CrcMismatch => write!(f, "CRC doesn't match"),
            Error::PzemError => write!(f, "Internal PZEM004T error"),
            Error::IllegalAddress => write!(f, "Illegal address"),
            Error::WriteError(e) => write!(f, "Could not write: {}", e),
            Error::ReadError(e) => write!(f, "Could not read: {}", e),
        }
    }
}

// 16-bit cyclic redundancy check (CRC).
fn crc_write(buf: &mut [u8]) {
    let n = buf.len();
    let crc = u16::to_be(crc16::State::<crc16::MODBUS>::calculate(&mut buf[0..n - 2]));

    buf[n - 2] = (crc >> 8) as u8;
    buf[n - 1] = (crc >> 0) as u8;
}

fn crc_check(buf: &[u8]) -> bool {
    let n = buf.len();
    let crc = u16::from_be(crc16::State::<crc16::MODBUS>::calculate(&buf[0..n - 2]));

    (crc >> 8) as u8 == buf[n - 2] && crc as u8 == buf[n - 1]
}

fn result_convert(buf: &[u8; 25], m: &mut Measurement) {
    m.voltage = (((buf[3] as u16) << 8) | buf[4] as u16) as f32 / 10.0;
    m.current = (((buf[5] as u32) << 8)
        | ((buf[6] as u32) << 0)
        | ((buf[7] as u32) << 24)
        | ((buf[8] as u32) << 16)) as f32
        / 1000.0;
    m.power = (((buf[9] as u32) << 8)
        | ((buf[10] as u32) << 0)
        | ((buf[11] as u32) << 24)
        | ((buf[12] as u32) << 16)) as f32
        / 10.0; // TODO something is wrong
    m.energy = (((buf[13] as u32) << 8)
        | ((buf[14] as u32) << 0)
        | ((buf[15] as u32) << 24)
        | ((buf[16] as u32) << 16)) as f32
        / 1000.0;
    m.frequency = (((buf[17] as u16) << 8) | ((buf[18] as u16) << 0)) as f32 / 10.0;
    m.pf = (((buf[19] as u16) << 8) | ((buf[20] as u16) << 0)) as f32 / 100.0;
    m.alarm = (((buf[21] as u16) << 8) | ((buf[22] as u16) << 0)) != 0;
}

/// Measurement results stored as the 32-bit floating point variables.
#[derive(Debug, Default, Copy, Clone)]
pub struct Measurement {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
    pub energy: f32,
    pub frequency: f32,
    pub pf: f32,
    pub alarm: bool,
}

/// Struct representing a PZEM004T sensor connected to a serial bus.
pub struct Pzem<Serial> {
    uart: Serial,
    addr: u8,
}

impl<Serial, WriteError, ReadError> Pzem<Serial>
where
    Serial: serial::Write<u8, Error = WriteError> + serial::Read<u8, Error = ReadError>,
{
    /// Creates a new PZEM004T struct, consuming the serial peripheral.
    ///
    /// When omitting the `addr` argument, will use the default general address for a
    /// single-slave environment, namely `0xf8`.
    ///
    /// Can return `Err(Error::IllegalAddress)` if `addr` is not in range of legal addresses `[0x01..0xf8]`.
    pub fn new(uart: Serial, addr: Option<u8>) -> Result<Self, Error<WriteError, ReadError>> {
        let addr = addr.unwrap_or(ADDR_DEFAULT);
        if addr != ADDR_DEFAULT && (addr < ADDR_MIN || addr > ADDR_MAX) {
            return Err(Error::IllegalAddress);
        }

        Ok(Self { uart, addr })
    }

    fn communicate<T: timer::CountDown>(
        &mut self,
        req: &[u8],
        resp: &mut [u8],
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<(), Error<WriteError, ReadError>> {
        // Make sure the input queue is empty before sending the request.
        self.uart.drain().map_err(Error::ReadError)?;

        self.uart.write_blocking(&req).map_err(Error::WriteError)?;
        block!(self.uart.flush()).map_err(Error::WriteError)?;

        if self
            .uart
            .read_blocking(timeout, resp)
            .map_err(Error::ReadError)?
            < resp.len() as u8
        {
            // If read_blocking has written less than N bytes,
            // we had a timeout.
            return Err(Error::TimedOut);
        }

        // First two bytes of the response (slave addr. + function code)
        // must correspond to the request.
        if resp[0] != req[0] || resp[1] != req[1] {
            return Err(Error::PzemError);
        }

        // If the response length is just 4 bytes, it is faster to compare
        // with the request CRC, as they are exactly the same.
        if resp.len() == 4 && (resp[2] != req[2] || resp[3] != req[3]) {
            return Err(Error::CrcMismatch);
        }

        if !crc_check(&resp) {
            return Err(Error::CrcMismatch);
        }

        Ok(())
    }

    /// Reads the measurements off the sensor and stores them into `m`.
    ///
    /// The timeout can be omitted (will wait indefinitely) in such a way:
    ///
    ///     pzem.communicate::<NoTimeout>(&mut m, None).unwrap();
    ///
    /// Look [`NoTimeout`](struct.NoTimeout.html).
    pub fn read<T: timer::CountDown>(
        &mut self,
        m: &mut Measurement,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<(), Error<WriteError, ReadError>> {
        let mut buf = [
            self.addr,              // Slave address
            CMD_READ,               // Function code: read measurement result
            0,                      // Register address high byte
            0,                      // Register address low byte
            (REG_COUNT >> 8) as u8, // Number of registers to be read.
            (REG_COUNT >> 0) as u8, // Number of registers to be read.
            0,                      // CRC
            0,                      // CRC
        ];

        crc_write(&mut buf);

        // The response: slave address + CMD_RIR + number of bytes + 20 bytes + CRC + CRC
        let mut resp: [u8; 25] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        result_convert(&resp, m);

        Ok(())
    }

    /// Reads the current power alarm threshold value of the energy monitor.
    ///
    /// In case of success, returns the raw `u16` value of the alarm threshold, where 1LSB corresponds to 1W.
    pub fn get_threshold<T: timer::CountDown>(
        &mut self,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<u16, Error<WriteError, ReadError>> {
        let mut buf = [
            self.addr,                    // Slave address
            CMD_READ_PARAM,               // Function code: read internal parameter
            (PARAM_THRESHOLD >> 8) as u8, // Parameter address
            (PARAM_THRESHOLD >> 0) as u8, // Parameter address
            0,                            // Number of registers to be read high byte.
            1,                            // Number of registers to be read low byte.
            0,                            // CRC
            0,                            // CRC
        ];

        crc_write(&mut buf);

        let mut resp: [u8; 7] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        Ok(((resp[3] as u16) << 8) | ((resp[4] as u16) << 0))
    }

    /// Reads the current Modbus-RTU address of the energy monitor.
    ///
    /// Returns the raw `u8` value of the address, or an error.
    pub fn get_addr<T: timer::CountDown>(
        &mut self,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<u16, Error<WriteError, ReadError>> {
        let mut buf = [
            self.addr,               // Slave address
            CMD_READ_PARAM,          // Function code: read internal parameter
            (PARAM_ADDR >> 8) as u8, // Parameter address
            (PARAM_ADDR >> 0) as u8, // Parameter address
            0,                       // Number of registers to be read high byte.
            1,                       // Number of registers to be read low byte.
            0,                       // CRC
            0,                       // CRC
        ];

        crc_write(&mut buf);

        let mut resp: [u8; 7] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        Ok(((resp[3] as u16) << 8) | ((resp[4] as u16) << 0))
    }

    /// Sets the power alarm threshold value of the energy monitor.
    ///
    /// # Example
    ///
    ///     // Will set the alarm threshold to 230 W:
    ///     pzem.set_threshold(230, Some(&mut ti, 2.hz())).unwrap();
    ///
    pub fn set_threshold<T: timer::CountDown>(
        &mut self,
        threshold: u16,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<(), Error<WriteError, ReadError>> {
        let mut buf: [u8; 8] = [
            self.addr,                    // Slave address
            CMD_WRITE_PARAM,              // Function code: set internal parameter
            (PARAM_THRESHOLD >> 8) as u8, // Threshold parameter register address
            (PARAM_THRESHOLD >> 0) as u8, // Threshold parameter register address
            (threshold >> 8) as u8,       // Threshold parameter value
            (threshold >> 0) as u8,       // Threshold parameter value
            0,                            // CRC
            0,                            // CRC
        ];

        crc_write(&mut buf);

        let mut resp: [u8; 8] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        Ok(())
    }

    /// Sets the Modbus-RTU address of the energy monitor.
    ///
    /// Also updates the [`Pzem`](struct.Pzem.html) struct to refer to the sensor by the new address.
    ///
    /// # Example
    ///
    ///     // Will set the slave address to 0x10:
    ///     pzem.set_addr(0x10, Some(&mut tim, 2.hz())).unwrap();
    ///
    pub fn set_addr<T: timer::CountDown>(
        &mut self,
        addr: u8,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<(), Error<WriteError, ReadError>> {
        if addr < ADDR_MIN || addr > ADDR_MAX {
            return Err(Error::IllegalAddress);
        }

        let mut buf: [u8; 8] = [
            self.addr,               // Slave address
            CMD_WRITE_PARAM,         // Function code: set internal parameter
            (PARAM_ADDR >> 8) as u8, // Slave address parameter reg.
            (PARAM_ADDR >> 0) as u8, // Slave address parameter reg.
            0,                       // High byte of the address reg. is always 0
            addr,                    // New slave address
            0,                       // CRC
            0,                       // CRC
        ];

        crc_write(&mut buf);

        let mut resp: [u8; 8] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        self.addr = addr;

        Ok(())
    }

    /// Sets the energy counting register back to 0.
    pub fn reset_energy<T: timer::CountDown>(
        &mut self,
        timeout: Option<(&mut T, T::Time)>,
    ) -> Result<(), Error<WriteError, ReadError>> {
        let mut buf = [self.addr, CMD_RESET, 0, 0];
        crc_write(&mut buf);

        let mut resp: [u8; 4] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
        self.communicate(&buf, &mut resp, timeout)?;

        Ok(())
    }

    /// Releases the underlying serial peripheral.
    pub fn release(self) -> Serial {
        self.uart
    }
}

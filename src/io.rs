use hal::serial;
use hal::timer;

pub trait WriteBlocking {
    type Error;
    fn write_blocking(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
}

impl<Uart: serial::Write<u8>> WriteBlocking for Uart {
    type Error = Uart::Error;
    fn write_blocking(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        for i in 0..buf.len() {
            block!(self.write(buf[i]))?;
        }

        Ok(())
    }
}

pub trait ReadBlocking {
    type Error;
    fn read_blocking<T: timer::CountDown>(
        &mut self,
        timeout: Option<(&mut T, T::Time)>,
        buf: &mut [u8],
    ) -> Result<u8, Self::Error>;
}

impl<Uart: serial::Read<u8>> ReadBlocking for Uart {
    type Error = Uart::Error;
    fn read_blocking<T: timer::CountDown>(
        &mut self,
        timeout: Option<(&mut T, T::Time)>,
        buf: &mut [u8],
    ) -> Result<u8, Self::Error> {
        let mut i = 0;
        if timeout.is_some() {
            let (timer, timeout) = timeout.unwrap();
            timer.start(timeout);

            while i < buf.len() {
                match timer.wait() {
                    Err(nb::Error::WouldBlock) => match self.read() {
                        Err(nb::Error::Other(e)) => return Err(e),
                        Err(nb::Error::WouldBlock) => continue,
                        Ok(b) => {
                            buf[i] = b;
                            i += 1;
                        }
                    },
                    // NOTE: the error type for wait() is Void.
                    Err(nb::Error::Other(_)) => unreachable!(),
                    Ok(()) => break, // timeout!
                }
            }
        } else {
            while i < buf.len() {
                buf[i] = block!(self.read())?;
                i += 1;
            }
        }

        Ok(i as u8)
    }
}

pub trait Drain {
    type Error;
    fn drain(&mut self) -> Result<(), Self::Error>;
}

impl<Uart: serial::Read<u8>> Drain for Uart {
    type Error = Uart::Error;
    fn drain(&mut self) -> Result<(), Self::Error> {
        loop {
            match self.read() {
                Err(nb::Error::WouldBlock) => return Ok(()),
                Err(nb::Error::Other(e)) => return Err(e),
                Ok(_) => continue,
            }
        }
    }
}

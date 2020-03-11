extern crate nb;

use hal::timer::CountDown;

pub struct NoTimeout;

impl CountDown for NoTimeout {
    type Time = ();
    fn start<T: Into<Self::Time>>(&mut self, _count: T) {} // no-op
    fn wait(&mut self) -> nb::Result<(), void::Void> {
        Err(nb::Error::WouldBlock)
    }
}

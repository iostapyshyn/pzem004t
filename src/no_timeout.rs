extern crate nb;

use hal::timer::CountDown;

/// Empty struct to satisfy Rust's type requirements, when using `None` for the `timeout` parameter.
/// # Example
/// ```rust
/// pzem.communicate::<NoTimeout>(&mut m, None);
/// ```
///
/// Will panic with `unreachable!()` if used incorrectly to signalize a programmer error.
pub struct NoTimeout;

impl CountDown for NoTimeout {
    type Time = ();
    fn start<T: Into<Self::Time>>(&mut self, _count: T) {
        unreachable!();
    }
    fn wait(&mut self) -> nb::Result<(), void::Void> {
        unreachable!();
    }
}

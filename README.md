# pzem004t
An embedded-hal driver for the PZEM004T Energy Monitor.

The driver must be initialized by passing a Serial interface peripheral
to the [`Pzem::new`](struct.Pzem.html#method.new)
function, which in turn will create a driver instance, with the slave address specified,
or the default general address for a single-slave environment `0xf8`.

## Examples

Examples can be found in the [`examples/`](https://github.com/iostapyshyn/pzem004t/tree/master/examples) directory.

### Read the measurements off the sensor every second

    let mut pzem = pzem004t::Pzem::new(serial, None).unwrap();
    let mut m = pzem004t::Measurement::default();
    loop {
        match pzem.read(&mut m, Some((&mut tim, TIMEOUT))) {
            Err(e) => hprintln!("Could not read PZEM004T: {}", e).unwrap(),
            Ok(()) => {
                hprintln!("Voltage: {:.1} V", m.voltage).unwrap();
                hprintln!("Current: {:.3} A", m.current).unwrap();
                hprintln!("Power: {:.1} W", m.power).unwrap();
                hprintln!("Energy: {:.3} kWh", m.energy).unwrap();
                hprintln!("Frequency: {:.1} Hz", m.frequency).unwrap();
                hprintln!("Power factor: {:.2}", m.pf).unwrap();
                hprintln!("Alarm: {}\n", m.alarm).unwrap();
            }
        }

        tim.start(1.hz());
        block!(tim.wait()).unwrap();
    }

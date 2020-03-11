#![no_std]
#![no_main]

extern crate pzem004t;

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Parity, Serial, StopBits},
    time::Hertz,
    timer::Timer,
};

const TIMEOUT: Hertz = Hertz(1);

#[entry]
fn main() -> ! {
    // Get access to the core and the device specific peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frequencies
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

    // USART1
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config {
            baudrate: 9_600.bps(),
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        },
        clocks,
        &mut rcc.apb2,
    );

    let mut tim = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());
    let mut pzem = pzem004t::Pzem::new(serial, None).unwrap();
    let mut m = pzem004t::Measurement::default();

    loop {
        match pzem.read(&mut m, Some((&mut tim, TIMEOUT))) {
            Err(e) => hprintln!("Could not read PZEM004T: {:?}", e).unwrap(),
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
}

//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::multicore::{Multicore, Stack};
// use bsp::hal::sio::SioFifo;
// use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use heapless::String;
use panic_probe as _;
use rp_pico as bsp;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { bsp::hal::pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin: bsp::hal::gpio::Pin<
        bsp::hal::gpio::bank0::Gpio25,
        bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
        bsp::hal::gpio::PullDown,
    > = pins.led.into_push_pull_output();

    let mut delay_duration: u32 = 500;

    loop {
        let input = sio.fifo.read();

        match input {
            Some(x) => delay_duration = x,
            _ => (),
        }

        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(delay_duration);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(delay_duration);
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // let timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // let mut said_hello: bool = false;

    let sys_freq = clocks.system_clock.freq().to_Hz();

    let mut sio = Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _temp = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });

    loop {
        // if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
        //     said_hello = true;
        //     let _ = serial.write(b"Hello, World!\r\n");

        //     let time = timer.get_counter().ticks();
        //     let mut text: String<64> = String::new();
        //     writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

        //     let _ = serial.write(text.as_bytes());
        // }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    // buf.iter_mut().take(count).for_each(|b| {
                    //     b.make_ascii_uppercase();
                    // });

                    let _temp = String::from_iter(Vec::from_iter(buf.iter()));
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];

                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}

// End of file

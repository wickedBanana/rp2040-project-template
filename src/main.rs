#![no_std]
#![no_main]
use bsp::entry;
use bsp::hal::multicore::{Multicore, Stack};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;
use rp_pico as bsp;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use bsp::hal::{
    adc::Adc,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn calc_temp(adc_counts: u16) -> f32 {
    let voltage: f32 = adc_counts as f32 * (3300f32 / 4096f32);
    info!("voltage {} adc_counts {}", voltage, adc_counts);
    let result = 27 as f32 - (voltage as i16 - 706) as f32 / 1.721;

    result
}

fn core1_task(sys_freq: u32) -> ! {
    info!("Core 1 start");
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

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temperature_sensor = adc.take_temp_sensor().unwrap();
    let mut delay_duration: u32 = 500;

    loop {
        if sio.fifo.is_read_ready() {
            delay_duration = sio.fifo.read().unwrap();
        }

        led_pin.set_high().unwrap();
        delay.delay_ms(delay_duration);
        led_pin.set_low().unwrap();
        delay.delay_ms(delay_duration);
        let temperature = calc_temp(adc.read(&mut temperature_sensor).unwrap());
        if sio.fifo.is_write_ready() {
            sio.fifo.write(temperature.to_bits())
        }
        info!("Temperature {}", temperature);
    }
}

#[entry]
fn main() -> ! {
    info!("Core 0 start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

    let sys_freq = clocks.system_clock.freq().to_Hz();

    let mut sio = Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });

    loop {
        let temperature: u32;

        if sio.fifo.is_read_ready() {
            temperature = sio.fifo.read().unwrap();
            info!("Temperature from fifo 1: {}", f32::from_bits(temperature));
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    debug!("Data received");
                    let mut i: u32 = 0;
                    let mut result: u32 = 0;
                    while i < count as u32 {
                        if buf[i as usize] < 0x30 || buf[i as usize] > 0x39 {
                            debug!("Bad char received: {}", buf[i as usize] as char);
                            result = 500;
                            break;
                        }

                        result = result
                            + (buf[i as usize] - 0x30) as u32
                                * u32::pow(10, (count as u32 - 1) - i);
                        i = i + 1;
                    }

                    if sio.fifo.is_write_ready() {
                        sio.fifo.write(result);
                    }

                    buf[count] = '\n' as u8;
                    let mut wr_ptr = &buf[..count + 1];

                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}

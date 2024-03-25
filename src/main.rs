use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_svc::hal::{
    gpio::{InterruptType, PinDriver, Pull},
    peripherals::Peripherals,
};

static PHOTOGATE_INT: AtomicBool = AtomicBool::new(false);

fn handle_photogate_raise() {
    PHOTOGATE_INT.store(true, Ordering::Relaxed);
}

fn main() -> ! {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut photodiode = PinDriver::input(peripherals.pins.gpio0).unwrap();
    photodiode.set_pull(Pull::Down).unwrap();
    photodiode
        .set_interrupt_type(InterruptType::PosEdge)
        .unwrap();
    unsafe {
        photodiode.subscribe(handle_photogate_raise).unwrap();
    }
    photodiode.enable_interrupt().unwrap();
    loop {
        if PHOTOGATE_INT.load(Ordering::Relaxed) {
            PHOTOGATE_INT.store(false, Ordering::Relaxed);
            log::info!("Photogate is high and interrupt was triggered");
            photodiode.enable_interrupt().unwrap();
        }
    }
}

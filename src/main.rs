use std::{
    sync::atomic::{AtomicBool, Ordering},
    thread::sleep,
    time::Duration,
};

use esp_idf_svc::hal::{
    gpio::{InterruptType, PinDriver, Pull},
    peripherals::Peripherals,
    timer::{config::Config, TimerDriver},
};

static PHOTOGATE_INT: AtomicBool = AtomicBool::new(false);
static BUTTON_INT: AtomicBool = AtomicBool::new(false);

fn handle_photogate_raise() {
    PHOTOGATE_INT.store(true, Ordering::Relaxed);
}

fn handle_button_press() {
    BUTTON_INT.store(true, Ordering::Relaxed);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FSM {
    Idle = 0,
    Prepare,
    Ready,
    TimerStarted,
    TimerEnded,
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

    let mut button = PinDriver::input(peripherals.pins.gpio9).unwrap();
    button.set_pull(Pull::Up).unwrap();
    button.set_interrupt_type(InterruptType::NegEdge).unwrap();
    unsafe {
        button.subscribe(handle_button_press).unwrap();
    }
    let timer_config = Config::new();
    let mut timer = TimerDriver::new(peripherals.timer00, &timer_config).unwrap();
    let mut state = FSM::Idle;
    loop {
        match state {
            FSM::Idle => {
                button.enable_interrupt().unwrap();
                // Wait for button press
                // If press, go to Prepare
                loop {
                    if BUTTON_INT.load(Ordering::Relaxed) {
                        BUTTON_INT.store(false, Ordering::Relaxed);
                        log::info!("Button was pressed!");
                        state = FSM::Prepare;
                        break;
                    }
                }
            }
            FSM::Prepare => {
                // Wait for laser to be aligned correctly
                // Photogate signal should remain low for extended period of time
                // Move to Ready once complete
                log::info!("Preparing photogate for timing ...");
                timer.set_counter(0).unwrap();

                let mut count = 0;
                loop {
                    if photodiode.is_low() {
                        count += 1;
                    } else {
                        log::info!("Beam was broken prematurely");
                        count = 0;
                    }
                    sleep(Duration::from_millis(100));
                    if count > 30 {
                        state = FSM::Ready;
                        break;
                    }
                }
            }
            FSM::Ready => {
                log::info!("Photogate is ready!");
                photodiode.enable_interrupt().unwrap();
                loop {
                    if PHOTOGATE_INT.load(Ordering::Relaxed) {
                        PHOTOGATE_INT.store(false, Ordering::Relaxed);
                        log::info!("Photogate beam was broken. Timer started");
                        timer.enable(true).unwrap();
                        state = FSM::TimerStarted;
                        photodiode.enable_interrupt().unwrap();
                        break;
                    }
                }
            }
            FSM::TimerStarted => {
                if PHOTOGATE_INT.load(Ordering::Relaxed) {
                    PHOTOGATE_INT.store(false, Ordering::Relaxed);
                    log::info!("Photogate beam was broken. Timer ended");
                    timer.enable(false).unwrap();
                    state = FSM::TimerEnded;
                }
            }
            FSM::TimerEnded => {
                // Display time. Wait until acknowledgement from button press.
                // Go back to Idle
                log::info!(
                    "Time: {}",
                    (timer.counter().unwrap() as f64) / (timer.tick_hz() as f64)
                );
                state = FSM::Idle;
            }
        }
    }
}

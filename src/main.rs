#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::ops::{Deref, DerefMut};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;
use esp_hal::gpio::{Gpio0, Gpio9, PullDown};
use esp_hal::{
    clock::ClockControl,
    embassy,
    gpio::{Input, PullUp},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_println::println;
use portable_atomic::{AtomicBool, Ordering};
use strum::EnumCount;


#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
enum FSM {
    Idle = 0,
    Prepare,
    Ready,
    TimerStart,
    TimerRunning,
    TimerEnd,
}

type ButtonPin = Gpio9<Input<PullUp>>;
type PhotodiodePin = Gpio0<Input<PullDown>>;

static PHOTODIODE_INPUT: AtomicBool = AtomicBool::new(false);
static PHOTODIODE_FLAG: AtomicBool = AtomicBool::new(false);

static BUTTON_INPUT: AtomicBool = AtomicBool::new(false);
static BUTTON_FLAG: AtomicBool = AtomicBool::new(false);

static STATE: Mutex<CriticalSectionRawMutex, FSM> = Mutex::new(FSM::Idle);

#[embassy_executor::task]
async fn reset_handler(mut button: ButtonPin) -> ! {
    loop {
        esp_println::println!("waiting for press");
        button.wait_for_falling_edge().await.unwrap();
        esp_println::println!("button press");
        let mut s = STATE.lock().await;
        *(s.deref_mut()) = FSM::Prepare;
    }
}

// FIXME: Potential bug when using wait_for_low and wait_for_high:
// A press could in theory trigger (low and then high) in between the await statements
// Check embassy docs if they have any solution to this
#[embassy_executor::task]
async fn handle_button(mut button: ButtonPin) -> ! {
    loop {
        button.wait_for_low().await.unwrap();
        BUTTON_INPUT.store(true, Ordering::Relaxed);
        // *(BUTTON_FLAG.lock().await.borrow_mut().deref_mut()) = true;
        button.wait_for_high().await.unwrap();
        BUTTON_INPUT.store(false, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn handle_photodiode(mut photodiode: PhotodiodePin) -> ! {
    loop {
        photodiode.wait_for_low().await.unwrap();
        PHOTODIODE_INPUT.store(true, Ordering::Relaxed);
        // *(PHOTODIODE_FLAG.lock().await.borrow_mut().deref_mut()) = true;
        photodiode.wait_for_high().await.unwrap();
        PHOTODIODE_INPUT.store(false, Ordering::Relaxed);
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(
        &clocks,
        esp_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let photodiode = io.pins.gpio0.into_pull_down_input();
    let button = io.pins.gpio9.into_pull_up_input();

    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    spawner.must_spawn(handle_button(button));
    spawner.must_spawn(handle_photodiode(photodiode));
    let mut start_time = None;
    let mut end_time = None;

    const PREPARTION_SAMPLE_COUNT: u64 = 60;
    let mut prepartion_sample_left = PREPARTION_SAMPLE_COUNT;
    loop {
        Timer::after(Duration::from_millis(1)).await;
        if BUTTON_FLAG.swap(false, Ordering::AcqRel) {
            *(STATE.lock().await.deref_mut()) = FSM::Prepare;
        }
        let state = STATE.lock().await.deref().clone();
        match state {
            FSM::Idle => {
                // Wait for button press
                if BUTTON_FLAG.swap(false, Ordering::AcqRel) {
                    // If press, go to Prepare
                    println!("Preparing photogate for timing ...");
                    println!("Hold still ...");
                }
            }
            FSM::Prepare => {
                // Wait for laser to be aligned correctly
                // Photogate signal should remain low for extended period of time
                // Move to Ready once complete
                loop {
                    if BUTTON_FLAG.load(Ordering::Relaxed) {
                        break;
                    }
                    if PHOTODIODE_INPUT.load(Ordering::Relaxed) {
                        prepartion_sample_left -= 1;
                    } else {
                        println!("Beam was broken prematurely");
                        prepartion_sample_left = PREPARTION_SAMPLE_COUNT;
                    }
                    const DELAY_MS_BETWEEN_SAMPLES: u64 = 100;
                    Timer::after_millis(DELAY_MS_BETWEEN_SAMPLES).await;
                    if prepartion_sample_left == 0 {
                        println!("Photogate is ready!");
                        *(STATE.lock().await.deref_mut()) = FSM::Ready;
                        break;
                    }
                }
                // photodiode.wait_for_low().await.unwrap();
            }
            FSM::Ready => {
                if PHOTODIODE_FLAG.swap(false, Ordering::AcqRel) {
                    println!("Photogate beam was broken. Timer started");
                    start_time = Some(Instant::now());
                    *(STATE.lock().await.deref_mut()) = FSM::TimerStart;
                }
            }
            FSM::TimerStart => {
                if !PHOTODIODE_INPUT.load(Ordering::Relaxed) {
                    *(STATE.lock().await.deref_mut()) = FSM::TimerRunning;
                }
            }
            FSM::TimerRunning => {
                // Update segment display

                if PHOTODIODE_FLAG.swap(false, Ordering::Relaxed) {
                    println!("Photogate beam was broken. Timer ended");
                    end_time = Some(Instant::now());
                    *(STATE.lock().await.deref_mut()) = FSM::TimerEnd;
                }
            }
            FSM::TimerEnd => {
                // Display time.

                if let (Some(start), Some(end)) = (start_time, end_time) {
                    let duration = end.duration_since(start);
                    const SECS_TO_MILLIS: u32 = 1000;
                    println!(
                        "Time: {}",
                        (duration.as_millis() as f64) / (SECS_TO_MILLIS as f64)
                    );
                    start_time = None;
                    end_time = None;
                } else {
                    println!("Time was not able to be calculated!");
                }
                *(STATE.lock().await.deref_mut()) = FSM::Idle;
            }
        }
    }
}

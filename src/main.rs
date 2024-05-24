#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::ops::{Deref, DerefMut};
use core::pin::pin;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::{
    channel::{Channel, Sender},
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;
use esp_hal::gpio::{Gpio0, Gpio9, PullDown};
use esp_hal::i2c;
use esp_hal::{
    clock::ClockControl,
    embassy,
    gpio::{Input, PullUp},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_println::println;
use futures::future::{select, Either};
use strum::EnumCount;

pub mod display;
use display::I2C7SegDisplay;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
enum FSMStates {
    Idle = 0,
    Prepare,
    Ready,
    TimerStart,
    TimerRunning,
    TimerEnd,
}

type ButtonPin = Gpio9<Input<PullUp>>;
type PhotodiodePin = Gpio0<Input<PullDown>>;
type Photodiode = Mutex<CriticalSectionRawMutex, Option<PhotodiodePin>>;

static PHOTODIODE_INPUT: Photodiode = Mutex::new(None);

const DISPLAY_LENGTH: usize = 4;

static STATE: Mutex<CriticalSectionRawMutex, FSMStates> = Mutex::new(FSMStates::Idle);

const SECS_TO_MILLIS: u32 = 1000;

static DISPLAY_TIMES: Signal<CriticalSectionRawMutex, (Instant, Option<Instant>)> = Signal::new();
const INPUT_CHANNEL_BUFFER_SIZE: usize = 64;
static INPUT_CHANNEL: Channel<CriticalSectionRawMutex, InputEvent, INPUT_CHANNEL_BUFFER_SIZE> =
    Channel::new();

static PHOTOGATE_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

enum InputEvent {
    ButtonDown,
    ButtonUp,
    PhotodiodeOpen,
    PhotodiodeClosed,
}

// FIXME: Potential bug when using wait_for_low and wait_for_high:
// A press could in theory trigger (low and then high) in between the await statements
// Check embassy docs if they have any solution to this
#[embassy_executor::task]
async fn handle_button(
    mut button: ButtonPin,
    channel: Sender<'static, CriticalSectionRawMutex, InputEvent, INPUT_CHANNEL_BUFFER_SIZE>,
) -> ! {
    loop {
        button.wait_for_low().await.unwrap();
        channel.send(InputEvent::ButtonDown).await;
        button.wait_for_high().await.unwrap();
        channel.send(InputEvent::ButtonUp).await;
    }
}

#[embassy_executor::task]
async fn handle_photodiode(
    photodiode: &'static Photodiode,
    channel: Sender<'static, CriticalSectionRawMutex, InputEvent, INPUT_CHANNEL_BUFFER_SIZE>,
) -> ! {
    loop {
        {
            photodiode
                .lock()
                .await
                .as_mut()
                .unwrap()
                .wait_for_low()
                .await
                .unwrap();
        }
        channel.send(InputEvent::PhotodiodeClosed).await;
        {
            photodiode
                .lock()
                .await
                .as_mut()
                .unwrap()
                .wait_for_high()
                .await
                .unwrap();
        }
        channel.send(InputEvent::PhotodiodeOpen).await;
    }
}

#[embassy_executor::task]
async fn handle_segment_display(
    mut display: I2C7SegDisplay<DISPLAY_LENGTH>,
    times: &'static Signal<CriticalSectionRawMutex, (Instant, Option<Instant>)>,
) -> ! {
    loop {
        display
            .set_blinkrate(display::H16K33Blinkrate::BlinkOff)
            .await
            .unwrap();
        display.write_f64(0_f64, 2).await.unwrap();
        times.wait().await;
        let mut ticker = Ticker::every(Duration::from_millis(50));
        loop {
            if !times.signaled() {
                break;
            }
            if let Some((start, opt_end)) = times.try_take() {
                if let Some(end) = opt_end {
                    let diff = end - start;
                    display
                        .set_blinkrate(display::H16K33Blinkrate::BlinkHalfHz)
                        .await
                        .unwrap();
                    display.write_fixed(diff.as_millis(), 3, 0).await.unwrap();
                } else {
                    let curr = Instant::now();
                    let diff = curr - start;
                    let diff_secs = diff.as_millis() as f64 / SECS_TO_MILLIS as f64;
                    if diff_secs < 10_f64 {
                        display.write_f64(diff_secs, 2).await.unwrap();
                    } else {
                        display.write_f64(diff_secs, 1).await.unwrap();
                    }
                }
            };
            ticker.next().await;
        }
    }
}

#[embassy_executor::task]
async fn check_laser_is_aligned(
    photodiode: &'static Photodiode,
    is_ready: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        let mut lock = photodiode.lock().await;
        let photodiode = lock.as_mut().unwrap();
        match select(Timer::after_millis(3000), pin!(photodiode.wait_for_low())).await {
            Either::Left((_, _)) => {
                is_ready.signal(());
                return;
            }
            Either::Right((_, _)) => {
                println!("Beam was broken prematurely");
                ticker.next().await;
            }
        };
    }
}

#[main]
async fn main(spawner: Spawner) {
    unsafe {
        // freeze occurs when using this
        // memory range for heap
        // ALLOCATOR.init(0x3FC8_0000 as *mut u8, 192 * 1024);
        ALLOCATOR.init(0x5000_0000 as *mut u8, 4 * 1024);
    }
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(
        &clocks,
        esp_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let photodiode = io.pins.gpio0.into_pull_down_input();
    PHOTODIODE_INPUT.lock().await.replace(photodiode);
    let button = io.pins.gpio9.into_pull_up_input();

    let i2c = i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        100_u32.kHz(),
        &clocks,
    );
    let display = I2C7SegDisplay::<DISPLAY_LENGTH>::new(0, i2c);

    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    spawner.must_spawn(handle_button(button, INPUT_CHANNEL.sender()));
    spawner.must_spawn(handle_photodiode(&PHOTODIODE_INPUT, INPUT_CHANNEL.sender()));
    spawner.must_spawn(handle_segment_display(display, &DISPLAY_TIMES));
    let mut start_time = None;
    let mut end_time = None;

    let mut ticker = Ticker::every(Duration::from_millis(1));

    let mut photogate_is_closed = false;
    let mut button_is_down = false;
    loop {
        // Use embassy_sync::channel::Channel and have here receive messages from input tasks
        // This ensures events and signals from inputs are all handled and none are skipped.
        //
        // When here in loop, empty the channel
        // Why?
        // 1. We want to ensure all inputs for the given timestamp are handled and are up-to-date
        // 2. Keeps the channel buffer empty ensuring input tasks do not get blocked leading to an
        // unresponsive device
        while let Ok(input) = INPUT_CHANNEL.try_receive() {
            // TODO: How would we detect button presses vs holds? Is that absolutely necessary?
            // (probably)
            // e.g. case:
            // Channel [ButtonDown, PhotogateOpen, ButtonUp]
            // The button was pressed, but the final value of button_is_down will be false
            // However, we would want to move to the Prepare state due to the press
            // Same thing *can* happen with the photodiode (rare but not infeasible)
            match input {
                InputEvent::ButtonDown => {
                    button_is_down = true;
                }
                InputEvent::ButtonUp => {
                    button_is_down = false;
                }
                InputEvent::PhotodiodeClosed => {
                    photogate_is_closed = true;
                }
                InputEvent::PhotodiodeOpen => {
                    photogate_is_closed = false;
                }
            }
        }

        let state = *STATE.lock().await.deref();
        match state {
            FSMStates::Idle => {
                // Wait for button press
                if button_is_down {
                    // If press, go to Prepare
                    match spawner.spawn(check_laser_is_aligned(&PHOTODIODE_INPUT, &PHOTOGATE_READY))
                    {
                        Ok(_) => {
                            *(STATE.lock().await.deref_mut()) = FSMStates::Prepare;
                            println!("Preparing photogate for timing ...");
                            println!("Hold still ...");
                        }
                        Err(e) => println!("Error occurred {:?}", e),
                    }
                }
            }
            FSMStates::Prepare => {
                // Wait for laser to be aligned correctly
                // Photogate signal should remain low for extended period of time
                // Move to Ready once complete
                if PHOTOGATE_READY.signaled() {
                    println!("Photogate is ready!");
                    PHOTOGATE_READY.reset();
                    *(STATE.lock().await.deref_mut()) = FSMStates::Ready;
                }
            }
            FSMStates::Ready => {
                if !photogate_is_closed {
                    start_time = Some(Instant::now());
                    println!("Photogate beam was broken. Timer started");

                    DISPLAY_TIMES.signal((start_time.unwrap(), None));
                    *(STATE.lock().await.deref_mut()) = FSMStates::TimerStart;
                }
            }
            FSMStates::TimerStart => {
                if photogate_is_closed {
                    *(STATE.lock().await.deref_mut()) = FSMStates::TimerRunning;
                }
            }
            FSMStates::TimerRunning => {
                // Update segment display

                if !photogate_is_closed {
                    end_time = Some(Instant::now());
                    println!("Photogate beam was broken. Timer ended");
                    if let Some(start_time) = start_time {
                        DISPLAY_TIMES.signal((start_time, end_time));
                        *(STATE.lock().await.deref_mut()) = FSMStates::TimerEnd;
                    }
                }
            }
            FSMStates::TimerEnd => {
                // Display time.

                if let (Some(start), Some(end)) = (start_time, end_time) {
                    let duration = end.duration_since(start);
                    println!(
                        "Time: {}",
                        (duration.as_millis() as f64) / (SECS_TO_MILLIS as f64)
                    );
                    start_time = None;
                    end_time = None;
                } else {
                    println!("Time was not able to be calculated!");
                }
                *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
            }
        }
        ticker.next().await;
    }
}

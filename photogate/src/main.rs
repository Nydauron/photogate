#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::ops::{Deref, DerefMut};
use core::pin::pin;
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::{
    channel::{Channel, Sender},
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::gpio::{Gpio0, Gpio4, Gpio9, Level, Output, Pull};
use esp_hal::peripherals::I2C0;
use esp_hal::system::SystemControl;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io},
    peripherals::Peripherals,
    prelude::*,
    timer::systimer::SystemTimer,
};
use esp_hal::{i2c, Async};
use esp_println as _;
use futures::future::{select, Either};
use strum::EnumCount;

use drivers::display::ht16k33_7seg_display::{AsyncI2C7SegDisplay, H16K33Blinkrate};
use drivers::initalization::Initialized;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
enum FSMStates {
    Idle = 0,
    Prepare,
    Ready,
    TimerRunning,
    TimerEnd,
}

#[allow(dead_code)]
type LaserPin = Output<'static, Gpio4>;
type ButtonPin = Input<'static, Gpio9>;
type PhotodiodePin = Input<'static, Gpio0>;
type Photodiode = Mutex<CriticalSectionRawMutex, Option<PhotodiodePin>>;

static PHOTODIODE_INPUT: Photodiode = Mutex::new(None);

const DISPLAY_LENGTH: usize = 4;

static STATE: Mutex<CriticalSectionRawMutex, FSMStates> = Mutex::new(FSMStates::Idle);

const SECS_TO_MILLIS: u32 = 1000;
const SECS_TO_MICROS: u32 = 1_000_000;

static DISPLAY_COMMAND: Signal<CriticalSectionRawMutex, DisplayCommand> = Signal::new();
const INPUT_CHANNEL_BUFFER_SIZE: usize = 64;
static INPUT_CHANNEL: Channel<CriticalSectionRawMutex, InputEvent, INPUT_CHANNEL_BUFFER_SIZE> =
    Channel::new();
static TIMER_SIGNAL: Signal<CriticalSectionRawMutex, TimerSignal> = Signal::new();
static CANCEL_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static PHOTOGATE_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

enum InputEvent {
    ButtonDown,
    ButtonUp,
}

#[derive(Debug, Clone, Copy)]
enum DisplayCommand {
    #[allow(dead_code)]
    Clear,
    Zero,
    Pending,
    StartTimer(Instant),
    EndTimer {
        start: Instant,
        end: Instant,
    },
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
        button.wait_for_low().await;
        channel.send(InputEvent::ButtonDown).await;
        button.wait_for_high().await;
        channel.send(InputEvent::ButtonUp).await;
    }
}

enum TimerSignal {
    StartTimer(Instant),
    EndTimer(Instant, Instant),
}
#[embassy_executor::task]
async fn handle_photodiode(
    photodiode: &'static Photodiode,
    signal_timer: &'static Signal<CriticalSectionRawMutex, TimerSignal>,
    cancel: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    let starttime = {
        let mut lock = photodiode.lock().await;
        let photodiode = lock.as_mut().unwrap();
        #[allow(clippy::let_and_return)]
        let start = match select(pin!(photodiode.wait_for_high()), cancel.wait()).await {
            Either::Left((_, _)) => {
                let start = Instant::now();
                signal_timer.signal(TimerSignal::StartTimer(start));
                debug!("Start signaled: {}", start);
                start
            }
            Either::Right((_, _)) => {
                cancel.reset();
                return;
            }
        };
        start
    };
    {
        let mut lock = photodiode.lock().await;
        let photodiode = lock.as_mut().unwrap();
        debug!("Waiting for laser beam to reconnect ...");
        match select(pin!(photodiode.wait_for_low()), cancel.wait()).await {
            Either::Left((_, _)) => {}
            Either::Right((_, _)) => {
                cancel.reset();
                return;
            }
        };
    }
    {
        let mut lock = photodiode.lock().await;
        let photodiode = lock.as_mut().unwrap();
        debug!("Waiting for laser beam to break ...");
        match select(pin!(photodiode.wait_for_high()), cancel.wait()).await {
            Either::Left((_, _)) => {
                let endtime = Instant::now();
                signal_timer.signal(TimerSignal::EndTimer(starttime, endtime));
                debug!("Stop signaled: {}", endtime);
            }
            Either::Right((_, _)) => {
                cancel.reset();
            }
        };
    }
}

#[embassy_executor::task]
async fn handle_segment_display(
    mut display: AsyncI2C7SegDisplay<DISPLAY_LENGTH, i2c::I2C<'static, I2C0, Async>, Initialized>,
    command: &'static Signal<CriticalSectionRawMutex, DisplayCommand>,
) -> ! {
    display.clear_display().await.unwrap();
    let (mut starttime, mut endtime) = (None, None);
    // FIX: Ideally, we should try to use a Ticker, but maybe we can make our own that doesn't try
    // to "catch up"
    const FAST_UPDATE: Duration = Duration::from_millis(16);
    const SLOW_UPDATE: Duration = Duration::from_millis(90);
    let mut update_delay = FAST_UPDATE;
    let mut pending_ticker = Ticker::every(Duration::from_millis(150));
    let mut latest_command = None;
    let mut is_timer_running = false;
    let mut pending_pos = 0;
    loop {
        let input_command = if is_timer_running {
            command.try_take()
        } else {
            Some(command.wait().await)
        };
        if let Some(cmd) = input_command {
            latest_command = Some(cmd);
            match cmd {
                DisplayCommand::Clear => {
                    is_timer_running = false;
                    (starttime, endtime) = (None, None);
                    display.clear_display().await.unwrap();
                    display
                        .set_blinkrate(H16K33Blinkrate::BlinkOff)
                        .await
                        .unwrap();
                }
                DisplayCommand::Zero => {
                    is_timer_running = false;
                    (starttime, endtime) = (None, None);
                    display.write_f64(0_f64, 2).await.unwrap();
                    display
                        .set_blinkrate(H16K33Blinkrate::BlinkOff)
                        .await
                        .unwrap();
                }
                DisplayCommand::Pending => {
                    pending_pos = 0;
                    is_timer_running = true;
                    pending_ticker.reset();
                    (starttime, endtime) = (None, None);
                    display
                        .set_blinkrate(H16K33Blinkrate::BlinkOff)
                        .await
                        .unwrap();
                }
                DisplayCommand::StartTimer(start) => {
                    is_timer_running = true;
                    update_delay = FAST_UPDATE;
                    (starttime, endtime) = (Some(start), None);
                }
                DisplayCommand::EndTimer { start, end } => {
                    is_timer_running = false;
                    (starttime, endtime) = (Some(start), Some(end));
                }
            }
        }

        if let Some(DisplayCommand::Pending) = latest_command {
            display.write_raw(&[0x01 << pending_pos; 4]).await.unwrap();
            pending_pos = (pending_pos + 1) % 6;
            pending_ticker.next().await;
        }
        match (starttime, endtime) {
            (Some(start), None) => {
                let curr = Instant::now();
                let diff = curr - start;
                let diff_secs = diff.as_millis() as f64 / SECS_TO_MILLIS as f64;
                if diff_secs < 100_f64 {
                    display.write_f64(diff_secs, 2).await.unwrap();
                } else {
                    update_delay = SLOW_UPDATE;
                    display.write_f64(diff_secs, 1).await.unwrap();
                }
                Timer::after(update_delay).await;
            }
            (Some(start), Some(end)) => {
                let diff = end - start;
                display
                    .set_blinkrate(H16K33Blinkrate::BlinkHalfHz)
                    .await
                    .unwrap();
                // TODO: Change back to write_fixed
                display
                    .write_f64(diff.as_millis() as f64 / 1000_f64, 2)
                    .await
                    .unwrap();
            }
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn check_laser_is_aligned(
    photodiode: &'static Photodiode,
    is_ready: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    const HOLD_TIME_MSECS: u64 = 3000;
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        let mut lock = photodiode.lock().await;
        let photodiode = lock.as_mut().unwrap();
        match select(
            Timer::after_millis(HOLD_TIME_MSECS),
            pin!(photodiode.wait_for_high()),
        )
        .await
        {
            Either::Left((_, _)) => {
                is_ready.signal(());
                return;
            }
            Either::Right((_, _)) => {
                info!("Beam was broken prematurely");
                ticker.next().await;
                ticker.reset();
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
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let systimer = SystemTimer::new_async(peripherals.SYSTIMER);
    esp_hal_embassy::init(&clocks, systimer);
    // let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    // let ble_init = esp_wifi::initialize(
    //     esp_wifi::EspWifiInitFor::Ble,
    //     timg0,
    //     rng::Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    //     &clocks,
    // )
    // .unwrap();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let photodiode = Input::new(io.pins.gpio0, Pull::Down);
    PHOTODIODE_INPUT.lock().await.replace(photodiode);
    let button = Input::new(io.pins.gpio9, Pull::Up);
    let mut laser = Output::new(io.pins.gpio4, Level::Low);
    laser.set_low();

    let i2c = i2c::I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        // 100_u32.kHz(),
        1_u32.MHz(),
        &clocks,
    );
    let display = AsyncI2C7SegDisplay::<DISPLAY_LENGTH, _, _>::new(0, i2c);

    let mut display: AsyncI2C7SegDisplay<DISPLAY_LENGTH, _, _> =
        display.initialize().await.unwrap();
    display.set_brightness(15).await.unwrap();

    spawner.must_spawn(handle_button(button, INPUT_CHANNEL.sender()));
    spawner.must_spawn(handle_segment_display(display, &DISPLAY_COMMAND));
    let mut start_time = None;
    let mut end_time = None;

    let mut ticker = Ticker::every(Duration::from_millis(1));

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
            }
        }

        let state = *STATE.lock().await.deref();
        match state {
            FSMStates::Idle => {
                laser.set_low();
                // Wait for button press
                if button_is_down {
                    // If press, go to Prepare
                    PHOTOGATE_READY.reset();
                    match spawner.spawn(check_laser_is_aligned(&PHOTODIODE_INPUT, &PHOTOGATE_READY))
                    {
                        Ok(_) => {
                            *(STATE.lock().await.deref_mut()) = FSMStates::Prepare;
                            DISPLAY_COMMAND.signal(DisplayCommand::Pending);
                            info!("Preparing photogate for timing ...");
                            info!("Hold still ...");
                        }
                        Err(e) => error!(
                            "Error occurred when trying to spawn photodiode preparation task {:?}",
                            e
                        ),
                    }
                }
            }
            FSMStates::Prepare => {
                // Wait for laser to be aligned correctly
                // Photogate signal should remain low for extended period of time
                // Move to Ready once complete
                laser.set_high();
                if PHOTOGATE_READY.signaled() {
                    TIMER_SIGNAL.reset();
                    CANCEL_SIGNAL.reset();
                    match spawner.spawn(handle_photodiode(
                        &PHOTODIODE_INPUT,
                        &TIMER_SIGNAL,
                        &CANCEL_SIGNAL,
                    )) {
                        Ok(_) => {
                            *(STATE.lock().await.deref_mut()) = FSMStates::Ready;
                            PHOTOGATE_READY.reset();
                            DISPLAY_COMMAND.signal(DisplayCommand::Zero);
                            info!("Photogate is ready!");
                        }
                        Err(e) => error!(
                            "Error occurred when trying to spawn photodiode task: {:?}",
                            e
                        ),
                    }
                }
            }
            FSMStates::Ready | FSMStates::TimerRunning => {
                laser.set_high();

                if button_is_down {
                    CANCEL_SIGNAL.signal(());
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                }
                if let Some(timer_event) = TIMER_SIGNAL.try_take() {
                    match timer_event {
                        TimerSignal::StartTimer(start) => {
                            info!("Photogate beam was broken. Timer started");
                            DISPLAY_COMMAND.signal(DisplayCommand::StartTimer(start));
                            *(STATE.lock().await.deref_mut()) = FSMStates::TimerRunning;
                        }
                        TimerSignal::EndTimer(start, end) => {
                            (start_time, end_time) = (Some(start), Some(end));
                            info!("Photogate beam was broken. Timer ended");
                            DISPLAY_COMMAND.signal(DisplayCommand::EndTimer { start, end });
                            *(STATE.lock().await.deref_mut()) = FSMStates::TimerEnd;
                        }
                    }
                }
            }
            FSMStates::TimerEnd => {
                laser.set_low(); // FIX: Keep beam on for ~1-2 seconds to allow for human
                                 // timing if needed

                // Display time
                if let (Some(start), Some(end)) = (start_time, end_time) {
                    let duration = end.duration_since(start);
                    info!(
                        "Time: {}",
                        (duration.as_micros() as f64) / (SECS_TO_MICROS as f64)
                    );
                    start_time = None;
                    end_time = None;
                } else {
                    warn!("Time was not able to be calculated!");
                }
                *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
            }
        }
        ticker.next().await;
    }
}

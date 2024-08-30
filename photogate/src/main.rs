#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use core::{
    cell::RefCell,
    ops::{Deref, DerefMut},
};

use critical_section::Mutex as CSMutex;
use defmt::{debug, info, warn};
use drivers::{
    display::ht16k33_7seg_display::{AsyncI2C7SegDisplay, H16K33Blinkrate},
    initalization::Initialized,
};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, pubsub::PubSubChannel,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Event, Gpio0, Gpio4, Gpio9, Input, Io, Level, Output, Pull},
    i2c,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    system::SystemControl,
    timer::systimer,
    Async,
};
use esp_println as _;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FSMStates {
    Idle,
    Prepare,
    Ready,
    TimerStart { start: Instant },
    TimerRunning { start: Instant },
    TimerEnd { start: Instant, end: Instant },
}

#[allow(dead_code)]
type LaserPin = Output<'static, Gpio4>;
type ButtonPin = Input<'static, Gpio9>;
type PhotodiodePin = Input<'static, Gpio0>;
type Photodiode = CSMutex<RefCell<Option<PhotodiodePin>>>;

static PHOTODIODE_INPUT: Photodiode = CSMutex::new(RefCell::new(None));

const DISPLAY_LENGTH: usize = 4;

static STATE: Mutex<CriticalSectionRawMutex, FSMStates> = Mutex::new(FSMStates::Idle);

const SECS_TO_MILLIS: u32 = 1000;
const SECS_TO_MICROS: u32 = 1_000_000;

static DISPLAY_COMMAND: Signal<CriticalSectionRawMutex, DisplayCommand> = Signal::new();
const INPUT_CHANNEL_BUFFER_SIZE: usize = 64;
static INPUT_CHANNEL: PubSubChannel<
    CriticalSectionRawMutex,
    InputType,
    INPUT_CHANNEL_BUFFER_SIZE,
    1,
    1,
> = PubSubChannel::new();
static CANCEL_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static PHOTOGATE_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(Debug, Clone, PartialEq, Eq)]
enum InputType {
    Button(InputLevel),
    Photodiode(InputLevel),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct InputLevel {
    ts: Instant,
    level: Level,
}

impl Ord for InputLevel {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.ts.cmp(&other.ts)
    }
}

impl PartialOrd for InputLevel {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
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

static BUTTON: CSMutex<RefCell<Option<ButtonPin>>> = CSMutex::new(RefCell::new(None));

#[handler]
fn handle_gpio() {
    debug!("GPIO interrupt");
    let now = Instant::now();
    // NOTE: This current implementation will override presses if the PubSub fills up. If there
    // is no more space to put another `InputLevel` into the PubSub, the earliest item in the
    // channel (that has not been read by everyone yet) will be removed.
    //
    // The reason why we are opting for a eventually-consistent high availabilty is to minimize
    // the amount of time we spend in the interrupt handler. We cannot use yield since
    // interrupts are synchronous and we cannot block as that would freeze the entire program.
    // Hence, we opt for a synchronous, non-blocking approach.
    if let Some(new_button_state) = critical_section::with(|cs| {
        let mut button = BUTTON.borrow_ref_mut(cs);
        let button = button.as_mut().unwrap();
        let interrupt_fired = button.is_interrupt_set();
        let state = button.get_level();
        button.clear_interrupt();
        interrupt_fired.then_some(state)
    }) {
        debug!("button interrupt");
        match INPUT_CHANNEL
            .immediate_publisher()
            .try_publish(InputType::Button(InputLevel {
                ts: now,
                level: new_button_state,
            })) {
            Ok(_) => {}
            Err(_) => {
                warn!("Button: Input PubSub is full.");
            }
        }
    }
    if let Some(new_photodiode_state) = critical_section::with(|cs| {
        let mut photodiode = PHOTODIODE_INPUT.borrow_ref_mut(cs);
        let photodiode = photodiode.as_mut().unwrap();
        let interrupt_fired = photodiode.is_interrupt_set();
        let state = photodiode.get_level();
        photodiode.clear_interrupt();
        interrupt_fired.then_some(state)
    }) {
        debug!("photodiode interrupt");
        match INPUT_CHANNEL
            .immediate_publisher()
            .try_publish(InputType::Photodiode(InputLevel {
                ts: now,
                level: new_photodiode_state,
            })) {
            Ok(_) => {}
            Err(_) => {
                warn!("Diode: Input PubSub is full.");
            }
        }
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

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    heap_allocator!(32_168);
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(handle_gpio);
    let mut photodiode = Input::new(io.pins.gpio0, Pull::Down);
    critical_section::with(|cs| {
        photodiode.listen(Event::AnyEdge);
        PHOTODIODE_INPUT.replace(cs, Some(photodiode));
    });
    let mut button = Input::new(io.pins.gpio9, Pull::Up);
    critical_section::with(|cs| {
        button.listen(Event::AnyEdge);
        BUTTON.replace(cs, Some(button));
    });
    let mut laser = Output::new(io.pins.gpio4, Level::Low);
    laser.set_low();

    let i2c = i2c::I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        400_u32.kHz(),
        &clocks,
    );
    let display = AsyncI2C7SegDisplay::<DISPLAY_LENGTH, _, _>::new(0, i2c);

    let mut display: AsyncI2C7SegDisplay<DISPLAY_LENGTH, _, _> =
        display.initialize().await.unwrap();
    display.set_brightness(15).await.unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1));

    let mut button_is_down = false;
    let mut photodiode_state = critical_section::with(|cs| {
        (
            Instant::now(),
            PHOTODIODE_INPUT
                .borrow_ref(cs)
                .as_ref()
                .unwrap()
                .get_level(),
        )
    });
    if photodiode_state.1 == Level::Low {
        warn!("Photodiode is already high without the laser on. Is the diode exposed to light?");
    }
    let mut input_sub = INPUT_CHANNEL.subscriber().unwrap();
    let mut last_console_print = Instant::now();

    let systimer = systimer::SystemTimer::new(peripherals.SYSTIMER).split::<systimer::Target>();
    esp_hal_embassy::init(&clocks, systimer.alarm0);
    spawner.must_spawn(handle_segment_display(display, &DISPLAY_COMMAND));
    loop {
        ticker.next().await;
        // Use embassy_sync::channel::Channel and have here receive messages from input tasks
        // This ensures events and signals from inputs are all handled and none are skipped.
        //
        // When here in loop, empty the channel
        // Why?
        // 1. We want to ensure all inputs for the given timestamp are handled and are up-to-date
        // 2. Keeps the channel buffer empty ensuring input tasks do not get blocked leading to an
        // unresponsive device
        while let Some(input) = input_sub.try_next_message_pure() {
            // TODO: How would we detect button presses vs holds? Is that absolutely necessary?
            // (probably)
            // e.g. case:
            // Channel [ButtonDown, PhotogateOpen, ButtonUp]
            // The button was pressed, but the final value of button_is_down will be false
            // However, we would want to move to the Prepare state due to the press
            // Same thing *can* happen with the photodiode (rare but not infeasible)
            match input {
                InputType::Button(input) => match input.level {
                    Level::High => {
                        button_is_down = false;
                    }
                    Level::Low => {
                        button_is_down = true;
                    }
                },
                InputType::Photodiode(input) => {
                    photodiode_state = (input.ts, input.level);
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
                    if critical_section::with(|cs| {
                        PHOTODIODE_INPUT.borrow_ref(cs).as_ref().unwrap().is_low()
                    }) {
                        warn!("Photodiode is already high without the laser on. Is the diode exposed to light?");
                    }
                    // FIXME: Current implementation with button interrupt code and pubsub channel
                    // processing can cause a backlog of responses in the pubsub queue which
                    // affectively DoS the device
                    *(STATE.lock().await.deref_mut()) = FSMStates::Prepare;
                    DISPLAY_COMMAND.signal(DisplayCommand::Pending);
                    info!("Preparing photogate for timing ...");
                    info!("Hold still ...");
                    photodiode_state.0 = Instant::now();
                    photodiode_state.1 = Level::High;
                }
            }
            FSMStates::Prepare => {
                // Wait for laser to be aligned correctly
                // Photogate signal should remain low for extended period of time
                // Move to Ready once complete
                const CALIBRATION_TIME: Duration = Duration::from_secs(3);
                const PRINT_DELAY: Duration = Duration::from_millis(500);
                laser.set_high();
                if photodiode_state.1 == Level::Low
                    && photodiode_state.0 + CALIBRATION_TIME <= Instant::now()
                {
                    CANCEL_SIGNAL.reset();

                    *(STATE.lock().await.deref_mut()) = FSMStates::Ready;
                    PHOTOGATE_READY.reset();
                    DISPLAY_COMMAND.signal(DisplayCommand::Zero);
                    info!("Photogate is ready!");
                    continue;
                }
                let now = Instant::now();
                if last_console_print + PRINT_DELAY <= now
                    && critical_section::with(|cs| {
                        PHOTODIODE_INPUT.borrow_ref(cs).as_ref().unwrap().is_high()
                    })
                {
                    info!("Beam was broken prematurely");
                    last_console_print = now;
                }
            }
            FSMStates::Ready => {
                laser.set_high();

                if button_is_down {
                    CANCEL_SIGNAL.signal(());
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                    continue;
                }
                if photodiode_state.1 == Level::High {
                    info!("Photogate beam was broken. Timer started");
                    let start = photodiode_state.0;
                    DISPLAY_COMMAND.signal(DisplayCommand::StartTimer(start));
                    *(STATE.lock().await.deref_mut()) = FSMStates::TimerStart { start };
                }
            }
            FSMStates::TimerStart { start } => {
                laser.set_high();
                if button_is_down {
                    CANCEL_SIGNAL.signal(());
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                    continue;
                }
                if photodiode_state.1 == Level::Low {
                    *(STATE.lock().await.deref_mut()) = FSMStates::TimerRunning { start };
                }
            }
            FSMStates::TimerRunning { start } => {
                laser.set_high();

                if button_is_down {
                    CANCEL_SIGNAL.signal(());
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                }
                if photodiode_state.1 == Level::High {
                    info!("Photogate beam was broken. Timer ended");
                    let end = photodiode_state.0;
                    // Display time
                    DISPLAY_COMMAND.signal(DisplayCommand::EndTimer { start, end });
                    let duration = end.duration_since(start);
                    info!(
                        "Time: {}",
                        (duration.as_micros() as f64) / (SECS_TO_MICROS as f64)
                    );
                    *(STATE.lock().await.deref_mut()) = FSMStates::TimerEnd { start, end };
                }
            }
            FSMStates::TimerEnd { start: _, end } => {
                const LASER_ON_DELAY: u64 = 3;
                if button_is_down {
                    CANCEL_SIGNAL.signal(());
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                }
                if end + Duration::from_secs(LASER_ON_DELAY) <= Instant::now() {
                    // Keep beam on for ~1-2 seconds to allow for human timing if needed
                    laser.set_low();
                    *(STATE.lock().await.deref_mut()) = FSMStates::Idle;
                }
            }
        }
    }
}

# Adaptive Traffic Signal Controller (STM32)

A four-lane traffic signal that changes how long each lane stays green based on how many vehicles are waiting. It runs on an STM32F103C8T6 (Blue Pill), written in bare-metal C with no HAL and no RTOS. We built it as a mini project for the Microprocessor Lab at MIT Manipal.

## Why we built it

Most junction signals run on fixed timers, so an empty lane gets the same green as a packed one. We wanted to see how far a cheap microcontroller and a few IR sensors could get toward fixing that, while also handling pedestrians and emergency vehicles, which fixed timers mostly ignore.

## What it does

- Counts vehicles in and out of each lane with two IR sensors per lane, and keeps a running queue count.
- Sets each lane's green time from its queue length, between 5 and 15 seconds.
- Cycles through the lanes in order (1, 2, 3, 4). Each lane goes green, then yellow for 2 s, then a 1 s all-red gap before the next lane.
- Lets pedestrians request a crossing. Pressing the button three times quickly switches to a longer "elderly" crossing time.
- Lets an emergency button give one lane priority.
- Shows a countdown on a 7-segment display for each lane, and overall status (active lane, phase, queue lengths) on a 16x2 LCD.
- Prints queue counts over UART so we could watch what the sensors were seeing while testing.

## How the timing works

Each lane has an IN sensor at the start of the stretch and an OUT sensor at the stop line. A vehicle is counted once, when the debounced sensor goes from clear to blocked (3 samples, 20 ms apart). The queue is updated like this:

```
Q = max(0, min(20, Q + IN - OUT))
```

When a lane turns green, its green time is:

```
green = min(15, 5 + 2 * Q)   (seconds)
```

So the green time grows with the queue and reaches the 15 s cap at 5 waiting vehicles:

| Queue | 0 | 1 | 2 | 3 | 4 | 5+ |
|-------|---|---|---|---|---|----|
| Green (s) | 5 | 7 | 9 | 11 | 13 | 15 |

The 2 seconds per vehicle is just a starting value. We haven't tuned it properly.

### Pedestrians and emergencies

The buttons trigger external interrupts, but the interrupt handlers only set a flag. The main loop checks the flags at safe points, so a request never cuts in during a yellow phase. In practice the current lane finishes its green, yellow and all-red sequence first, and then the request is served.

- **Pedestrian:** one press makes the walk phase at least 9 s. Three presses within 3 s makes it 12 s. The buzzer beeps once on a press and three times when elderly mode kicks in.
- **Emergency:** the chosen lane gets 12 s of green, and its display counts down from `E 12`.

### Timing without delays

Everything runs off the 1 ms SysTick timer and a small state machine, so there are no blocking `delay()` calls. That's what lets the displays, buttons, sensors and UART keep working while a phase is running.

## Hardware

- STM32F103C8T6 Blue Pill, programmed over SWD with an ST-Link V2
- 12 LEDs (red, yellow, green for each of 4 lanes) with 220-330 ohm resistors
- 8 IR obstacle-detection modules (IN and OUT for each lane)
- 4 TM1637 7-segment display modules, one per lane
- 16x2 LCD with an I2C backpack
- 2 PCF8574T I2C expanders (used for the lane 3 and 4 sensors and the pedestrian buttons)
- 8 push buttons (emergency and pedestrian), 1 buzzer
- UART-to-TTL module for the serial monitor

One thing worth mentioning: we used active IR obstacle modules instead of PIR sensors. A PIR sensor reacts to moving warm bodies, so it wouldn't pick up a vehicle sitting still at the stop line.

We also switched the debug interface to SWD-only so that PB3 and PB4 could be used as button inputs.

## Software

- Language: C, using CMSIS register definitions directly
- IDE: STM32CubeIDE
- Flashing: STM32CubeProgrammer with the ST-Link
- Serial output: 115200 baud, 8N1, sent as `L1=..,L2=..,L3=..,L4=..`

## Build and run

1. Clone the repo:
   ```
   git clone https://github.com/Saksham2607/adaptive-traffic-signal-stm32.git
   ```
2. In STM32CubeIDE, go to File > Open Projects from File System and pick the cloned folder.
3. Build the project (Project > Build All).
4. Connect the Blue Pill through the ST-Link and flash it with STM32CubeProgrammer, or use Run > Debug.
5. Open a serial terminal at 115200 baud to see the queue counts.

## What we tested

- The four-lane LED sequence and the all-red gaps, on the hardware.
- That the signals keep running smoothly while the other tasks (displays, buttons, UART) are active.
- The queue counting and green-time logic, checked offline for different queue lengths. The firmware compiles with zero warnings.
- The LCD and all four TM1637 countdowns, to make sure they match the remaining time and don't disturb each other on the shared clock line.
- The emergency and pedestrian buttons, including the buzzer behaviour.

We did not do a proper measured comparison against a fixed-timer signal, so we can't put a number on the improvement in waiting time. The results above are functional checks, not performance measurements.

## Known limitations

- The queue is an estimate. If a sensor misses a vehicle or counts one twice, that lane's count stays off until the queue clears.
- IR sensors are sensitive to ambient light and how they're mounted. Debouncing helps but doesn't fully solve it.
- Lanes are always served in the same order. Only the length of the green changes, not which lane goes next.
- The queue cap (20), maximum green (15 s) and 2 s per vehicle are prototype values.
- It's a breadboard prototype and hasn't been tried on anything like real traffic.

## Ideas for next time

- Measure average waiting time against a fixed-timer baseline.
- Detect stuck or failed sensors and recalibrate the count periodically.
- Let the controller pick the busiest lane next instead of going round-robin.
- Add Wi-Fi and a dashboard for remote monitoring.
- Try camera-based counting alongside the IR sensors.
- Connect several junctions together.


## Repository layout

```
Inc/                     header files
Src/                     application source
Startup/                 startup code
STM32F103C8TX_FLASH.ld   linker script
```

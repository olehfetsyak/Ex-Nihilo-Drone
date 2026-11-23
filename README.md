# Drone v.0.0 Sim (Python)

by Mori

This stable simulator is **Sky Gadget No. 1: Euphrosyne Mirth Particle Windmill (name not final)**. The naming rules, such as they are, are in `NAMES.md`.

A flight simulator for our micro drone, laid out the same way as the software plan. `software-plan.png` is the original plan, and `software-plan_current.png` shows what is built today:

- **Drone**: ESP32-C3 Super Mini flight controller + Seeed Studio XIAO ESP32-S3 FPV camera
- **Macbook (Command Center)**: keyboard → Command Center → Safety Layer → Radio Bridge (ESP-NOW) → drone, with flight data and video coming back

You fly it with the keyboard, the same way the real Command Center will fly the drone. Every key is on a 65% board (we're using an Epomaker Magcore65 Lite), so no F-row or numpad is needed.

It is meant to feel like our first real flights, not like a game: the drone only knows what its IMU can tell it, the laptop only knows what the drone sends back, and the build is slightly imperfect.

## Run it

Needs Python 3.9 or newer. Tested on Python 3.9, 3.12 and 3.13 on Windows. It should run on macOS too, but that has not been tried yet.

```
pip install -r requirements.txt
python run_sim.py
```

Questions are answered in `FAQ.md`.

## Keys

| Key | Does |
|---|---|
| Enter | Arm. It never disarms |
| Space | DISARM (same as clicking the big red button) |
| W / S | Climb / descend. Let go to hover. First W after arming takes off |
| Q / E | Hover trim down / up 1%. Tap E if it sinks hands off, Q if it climbs |
| Arrows | Lean forward, back, left, right |
| A / D | Turn left / right |
| C | Camera: chase, drone camera (XIAO, with video delay), ground |
| L | Cut / restore the control link (tests the drone's failsafe) |
| T | Take control when the video is lost |
| V | Cut / restore the video (tests the safety layer's hover, then land) |
| 1 to 8 | Start a flight test (see below). 0 goes back to free flight |
| F | Full screen |
| R | Reset (or try the flight test again) |
| P | Pause |
| H | Show every key. The simulator waits while the panel is open |
| Esc | Quit |

The drone can't tell when it has landed. After you put it down with S the motors keep running until you disarm.

Arming needs both the link and the video. If the video drops in flight, the flight keys are locked out, the drone hovers, and the screen asks whether to take control. Press T and the keys are yours again without video. If nobody answers within `safety_hover_s` (5 s) it lands by itself; T still takes over during the landing. DISARM and Q/E always work.

## Flight tests

Press a number key to start one. Each shows pink rings to fly to and ends in PASS or FAIL. They run on top of whatever `--set` settings you started with, so you can fly the same test against a different build.

| Key | For | Test | To pass |
|---|---|---|---|
| 1 | hardware | Hover | Hover for 10 s: 0.5 to 2 m up, within 1.5 m of the pad, slower than 0.8 m/s. Tells you the hover throttle of this build |
| 2 | hardware | Weak battery | Battery at 25%: climb to 3 m and land softly without a brownout |
| 3 | hardware | Crooked build | Board 3° off level, motors 8% apart: stay within 2 m of the pad for 10 s |
| 4 | hardware | Breeze | 2 m/s wind with 0.5 m/s gusts: stay within 3 m of the pad for 10 s |
| 5 | software | Link drop | The link drops for 3 s once you are hovering above 4 m: take it back from the failsafe and land softly |
| 6 | software | Video drop | The video drops for good as you cross the ring at more than 2 m/s: take control (T) or let it land itself, softly |
| 7 | software | Lag | Commands 60 ms late, video 350 ms late: land inside the ring |
| 8 | software | Lost packets | 85% of packets lost both ways: land inside the ring |

The clock only runs while every condition is met, and the panel says what is missing when it isn't. A soft landing is a touchdown under 1.8 m/s, and a landing test only ends once you disarm. Some to try:

```
python run_sim.py --set mass_g=60                  # then 1: a heavier build needs E, and about 65% throttle
python run_sim.py --set regulator=ams1117          # then 2: fails with a brownout
python run_sim.py --set safety_hover_s=1           # then 6: it only waits 1 s for you before landing
python run_sim.py --tests                          # list them
```

## What to expect when flying

These come straight from test flights in the simulator, and the real drone should do the same:

- **It never sits still.** Hands off, it drifts sideways at around 0.1 m/s, because the board is never mounted dead level and the motors never match. There is nothing on the drone that can see the drift.
- **Hover is a guess.** The laptop holds one throttle number (`hover_throttle`) when you let go of W and S. If the real drone is 5 g heavier than we thought, it sinks back to the floor hands off; 5 g lighter and it climbs at about 1 m/s. Q and E fix it in flight. Measuring the real hover throttle is the first job.
- **Leaning forward loses height.** In fast forward flight it sinks slowly and needs taps of W.
- **Lean angle is the speed limit.** About 2.8 m/s with a 16° limit, 4.8 m/s with 32°. The real lean ends up a degree or two more than asked while it speeds up, because the accelerometer can't tell leaning from accelerating. `safety_max_tilt` (default 30°) is the only speed limit there is.
- **The battery sags.** Around 3.5 minutes of hovering on 300 mAh, and the throttle needed to hover goes from about 60% to almost 80% by the end.
- **Link lost.** The drone levels out, holds `failsafe_throttle`, comes down at about 1.6 m/s (about 3 m/s on a 25% battery, which is a hard landing), and cuts the motors after `failsafe_land_s` seconds whether or not it has reached the floor.
- **Video lost.** The flight keys stop working. The safety layer levels the drone, holds hover throttle and asks whether to take control (T); after 5 s with no answer it eases the drone down at about 1 m/s. It can't brake: from 4 m/s it coasts about 6 m before drag slows it to a crawl.
- **Low battery.** "LOW BATTERY" shows when the battery drops under 3.4 V in flight, with about 45 seconds of hover left.

## Change the drone

Settings match the real parts and change as parts get picked. Pass them with `--set`:

```
python run_sim.py --set regulator=ams1117          # watch it brown out on takeoff
python run_sim.py --set mass_g=60                  # built heavier than planned: needs E to hover
python run_sim.py --set hover_throttle=0.63        # the laptop's hover guess, once we've measured it
python run_sim.py --set level_error_deg=0 --set motor_mismatch=0   # a perfect build, for comparison
python run_sim.py --set safety_max_tilt=15         # gentler: lower lean and speed limit
python run_sim.py --set safety_hover_s=2           # video lost: wait 2 s for the pilot before landing
python run_sim.py --set wind=2
python run_sim.py --set video_loss=40               # stuttering video: 40% of frames never arrive
python run_sim.py --seed 1234                       # fly drone 1234 again
python run_sim.py --list                            # every setting, its default and its allowed range
```

A value outside the allowed range is refused with a message instead of starting.

Each launch is a different drone: the motor mismatch, the tilt of the board and the gyro bias are drawn fresh. The drone's number is in the window title and the H panel, and `--seed` with that number flies the same one again. The flight tests always use their own fixed drone.

The main ones are `chip` (c3, s3, esp32), `loop_hz`, `mass_g`, `motor_thrust_g`, `battery_mah`, `regulator` (ldo, ams1117, buckboost), `failsafe` (land, drop), `failsafe_throttle`, `link_delay_ms`, `link_loss`, `video_delay_ms`, `video_loss`, `cam_tilt`, `wind`, `gust` (both in m/s), `hover_throttle`, `safety_max_tilt`, `safety_hover_s`.

## Flight tuning

`--tune` flies a build through a fixed routine without a window (take off, hold 2 m, a sudden half-stick roll, a sudden half-stick turn) and prints how it responded, next to the default build:

```
python run_sim.py --tune --set p_roll=60 --set d_roll=40
```

| Line | What it tells you |
|---|---|
| hover throttle | The throttle this build hovers at, and the value to put in `hover_throttle` |
| hover wobble | How much it shakes hands off. More than a few deg/s means P or D is too high, or there is too much delay |
| motor spread | How far apart the motors are driven in the hover. A big spread means noise is getting through and the motors run hot |
| roll step | How fast it reaches a new lean, and how far it overshoots |
| yaw step | How fast it reaches a new turn rate |
| flight loop load | Whether the chip keeps up with `loop_hz` |

The tuning settings are `p_roll`, `i_roll`, `d_roll` (roll and pitch share them), `p_yaw`, `i_yaw`, `angle_gain`, `max_rate`, `yaw_rate`, `gyro_lpf`, `dterm_lpf` and `loop_hz`.

How far to trust it:

- Put the real build in first: `mass_g`, `motor_thrust_g` (thrust per motor), `arm_mm`, `battery_mah`. With guessed numbers the report describes a guessed drone.
- Use it to learn which way each value pushes and what bad tuning looks like, not to copy gains onto the drone. The simulator is more forgiving than real life: there is no frame flex, prop wash or bent prop.
- The gain numbers are on the simulator's own scale. They have not been checked against ESP-FC or Betaflight, so 45 here is not promised to be 45 there.

## What's simulated

### Drone side
- Physics at 4 kHz: thrust from motor speed squared, motor and ESC lag, drag, wind and gusts, ground contact and crashes
- An imperfect build: motors a few percent apart, the flight controller board about a degree off level
- Battery: voltage sag under load, the 3.3 V regulator, and brownout resets of the flight controller
- Flight controller like ESP-FC / Betaflight: gyro calibration, noisy gyro and accelerometer with motor vibration, filters, attitude estimate, self-levelling, rate PID, mixer with airmode, arming checks
- Failsafe like Betaflight's landing mode: level, fixed throttle, motors off after a set time
- Chip load: a slow chip can't run the flight loop as fast as asked
- XIAO camera view with video delay, frame rate and lost frames

### Command Center side
- **Radio Bridge**: ESP-NOW commands up and flight data down, each with packet rate, delay and loss
- **Flight data**: only what the drone can sense with the LSM6DSM: lean angles, battery voltage, armed. No height, speed or position
- **Command Center**: live video, flight data, link and video health, the big DISARM button
- **Safety Layer**: every command passes through it before the bridge
  - tilt limit (which is also the speed limit)
  - video lost → flight keys locked out, level out and hover, ask the pilot; no answer → land
  - the pilot always overrides: T takes control back, DISARM always works
  - link lost → the drone's own failsafe flies; the laptop keeps sending a calm hover command for when the link returns
  - no arming without both the link and the video
  - low battery warning

On screen, anything marked "sim only" (height, speed, CPU load, the pointer to the pad) is something the real Command Center won't have.

Not in the simulator: the AI and session recording from the plan.

## Later: the AI box

Three placeholders in `experiments/` hold notes only, and the simulator doesn't use any of them:

- `AI_brain.py`: the plan's AI (a small vision network, gestures, follow me), what it is given and what has to be built first.
- `fly_brain.py`: a separate mode with a real fruit fly brain in the same place, trained in this simulator.
- `indoor/indoor_place.py`: a room to fly in, with walls and a ceiling, instead of the open field.

## Command flow

```
Keyboard → pilot → Safety Layer → Radio Bridge (ESP-NOW) → Drone
                        ↑                                     |
                        └──────── flight data, video ─────────┘
```

## Files

The hardware side has its own folder and its own guide, `drone_sim/hardware/README.md`.

```
run_sim.py                     window, keys, and main loop

drone_sim/hardware/            the drone (hardware side)
  parts.py                     the build: every part setting, its default and allowed range
  drone.py                     physics, battery, flight controller, radio link (no graphics)
  flight_tests.py              flight tests 1 to 4
  tuning.py                    the --tune report
  README.md                    guide to this folder

drone_sim/                     the laptop (software side)
  __init__.py                  the name of this stable simulator
  command_center.py            laptop side, wired like the plan
  pilot.py                     keyboard -> stick values
  safety.py                    safety layer: tilt limit, hover or land
  video.py                     video link: delay, frame rate, dropouts
  flight_tests.py              runs the flight tests, and holds tests 5 to 8
  params.py                    laptop and weather settings, joined with the build
  view.py                      3D view and HUD (pygame)
  world.py                     obstacles
  quat.py                      quaternion math

tests/test_sim.py              flight checks that run without a window
tools/                         deeper checks: stress test, settings sweep, live window test (see tools/README.md)
FAQ.md                         common questions
NAMES.md                       how stable simulators are named
software-plan.png              the original plan
software-plan_current.png      the plan as built today
experiments/AI_brain.py        placeholder for the plan's AI (notes only)
experiments/fly_brain.py       placeholder for the fly brain mode (notes only)
experiments/indoor/            placeholder for an indoor place to fly (notes only)
```

## Tests

```
python tests/test_sim.py
```

That is the everyday check. `tools/` holds the deeper ones (random-input stress test, settings sweep, a live window test with real key presses); `tools/README.md` lists them.

## Notes

- Numbers like motor thrust, battery resistance, drag and chip loop cost are estimates until we measure the real parts. The drag guess is probably too high, so the real drone will likely fly and fall faster than this one; `drag_area_cm2` and `drag_linear` are the settings to correct.
- Tested with pygame 2.6.1, which is the version `requirements.txt` installs.
- All rights reserved. You're welcome to read the code; ask us before using or sharing it.
- The plan's Safety Layer has a speed limit and a hover-or-land fallback. With only an IMU the drone can't measure speed or height, so here the speed limit is a tilt limit and "hover" drifts. Doing either properly needs another sensor, such as optical flow with a range sensor.

# Hardware side

Everything about the physical drone lives in this folder. The laptop side (keyboard, Safety Layer, video, the window) is in the folder above and doesn't need touching for hardware changes.

| File | What it is | Change it when |
|---|---|---|
| `parts.py` | Every number that describes the build: weight, motors, battery, regulator, chip, tuning, radio, failsafe, sensors, camera | A part is picked, swapped or measured |
| `drone.py` | The model: physics, battery, flight controller, radio link | The model itself is wrong or missing something |
| `flight_tests.py` | Flight tests 1 to 4 | You want a new hardware test |
| `tuning.py` | The `--tune` report | You want another measurement in the report |

All commands run from the top folder of the repo.

## Measure these first

The defaults are estimates. The simulator only describes our drone once these are real:

| Setting | How to get it |
|---|---|
| `mass_g` | Weigh the finished drone with the battery in |
| `motor_thrust_g` | Thrust of one motor at full throttle on a full battery (thrust stand, or the motor's data sheet) |
| `arm_mm` | Centre of the frame to the centre of a motor |
| `battery_mah` | Printed on the battery |
| `regulator` | Which 3.3 V regulator is on the board: `ldo`, `ams1117` or `buckboost` |
| `chip`, `loop_hz` | The flight controller chip and the loop rate set in the firmware |
| `hover_throttle` | Fly flight test 1 (key 1), or run `--tune`: both report it. This one is a laptop setting, in `../params.py` |
| `drag_area_cm2`, `drag_linear` | Time the real drone over a measured distance at a held lean. Change these until the simulator flies the same distance in the same time at the same lean. Start with `drag_area_cm2`: the default is probably too high, so the simulator is slower than the real drone will be |

## Try a value without editing anything

```
python run_sim.py --set mass_g=58 --set regulator=ams1117     # fly that build
python run_sim.py --tune --set mass_g=58                      # tuning report for that build, no window
python run_sim.py --list                                      # every setting, its default and allowed range
```

Once a value is settled, change its default in `parts.py`.

## Add things

- **A regulator or chip:** add a line to `REGULATORS` or `CHIPS` in `parts.py`. It can then be chosen with `--set regulator=its_name`.
- **A setting:** give it a default in `PARTS` and a range in `LIMITS` (both in `parts.py`), then use it in `drone.py` as `p["its_name"]`. `--set` and `--list` pick it up by themselves.
- **A flight test:** the steps are at the top of `flight_tests.py`.

## Where things are in `drone.py`

| Part of the file | What it models |
|---|---|
| `reset` | Power-up state, and the random build errors (motor mismatch, board off level, gyro bias) |
| `_link_step` | The radio: packet rate, delay and loss, both ways |
| `_fc_loop` | The flight controller: gyro calibration, filters, attitude estimate, failsafe, arming, PID, mixer |
| `step` | Physics: battery and regulator, brownout, gyro and accelerometer, motors, forces, wind, ground |

## Check nothing broke

```
python tests/test_sim.py
```

## What the model leaves out

Frame flex, prop wash, bent props, motor heat, and anything the IMU can't sense. Drag is the same in every direction and doesn't change when the motors stop. Numbers like drag, battery resistance and chip loop cost are estimates until we measure them.

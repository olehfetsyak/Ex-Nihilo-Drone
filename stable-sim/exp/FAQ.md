# FAQ

## Flying

**Why won't it arm?**
Arming needs both the link and the video, the throttle low, and the drone level. The message at the bottom left says which one is missing. After a brownout or a failsafe, press Space and then Enter.

**Why does it drift when I'm not touching anything?**
The board is never mounted dead level and the motors never match, so it always leans a little. The drone only has an IMU, which can't see drift, so nothing corrects it. The real drone will do the same.

**Why does it keep climbing (or sinking) when I let go?**
With no height sensor, "hover" is one guessed throttle number. If the guess is a little high it climbs for good, a little low and it sinks. Tap Q or E until it stops. On the default drone the guess is already right, so any trim makes it worse.

**It won't take off at all.**
The build is heavier than the hover guess expects, and W only adds 10% on top of that guess. Tap E a few times, or start with a higher `hover_throttle`.

**The motors keep running after I land.**
The drone can't tell it has landed. Press Space to disarm.

**My keys stopped working in the air.**
The video was lost, so the Safety Layer locked the flight keys and is hovering. Press T to fly without video. If nobody does within 5 s it lands by itself. Space and Q/E always work.

**It fell out of the sky when the link dropped.**
That is the drone's own failsafe: it levels, holds a fixed throttle, and cuts the motors after `failsafe_land_s` seconds whether or not it has reached the floor. It can't see the ground.

**"Flight controller browned out"?**
The battery voltage sagged under load and the 3.3 V regulator couldn't hold up the flight controller. Try `--set regulator=ams1117` to see it happen on take-off, and `ldo` or `buckboost` to see it not.

**I flew off and can't find the pad.**
The "sim only" box at the bottom right points to the pad and gives the distance. Up in the box is ahead of the drone's nose. R resets.

**What does "sim only" mean?**
Something the simulator knows but the real Command Center won't: height, speed, position, CPU load.

## Flight tests

**The timer isn't counting.**
The clock only runs while every condition is met. The pink line in the test panel says what is missing, for example "too fast to call a hover".

**I landed but the test didn't finish.**
Landing tests only end once you disarm (Space).

**Is a test the same every time?**
Yes. Each test uses its own fixed drone and weather, so a retry is fair.

## Settings and builds

**How do I change the drone?**
`python run_sim.py --set name=value`, as many `--set` as you like. `--list` shows every setting, its default and its allowed range. To change a default for good, edit `drone_sim/hardware/parts.py`.

**Is it the same drone every time I start?**
No. Each launch is a different drone, with its own motor mismatch and board tilt. The number is in the window title and the H panel. `--seed 1234` flies drone 1234 again.

**Can I copy tuning values from here to the real drone?**
No. Use `--tune` to learn which way each value pushes. The physical numbers are estimates until measured, the gain scale has not been checked against ESP-FC, and the simulator is more forgiving than a real drone.

**Where does the hardware side live?**
In `drone_sim/hardware/`, with its own README.

## The project

**Why a keyboard and not a gamepad?**
The plan says gamepad, but we're flying with the keyboard for now. Every key is on a 65% board.

**Where are the AI and session recording from the plan?**
Not built. `experiments/AI_brain.py` and `experiments/fly_brain.py` are notes for later, and the simulator doesn't use them.

**What is the Euphrosyne Mirth Particle Windmill?**
The name of this stable simulator: Sky Gadget No. 1. It has no particles and no windmill. Every stable simulator gets a number and a grand name that explains nothing; `NAMES.md` has the rules.

**How do I check nothing is broken?**
`python tests/test_sim.py`. It flies the drone through every check without opening a window.

**What has not been tried yet?**
A person flying it by hand for long, and macOS. It has been run on Python 3.9, 3.12 and 3.13 on Windows.

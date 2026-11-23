# Tools

Extra checks for the simulator. None of them is needed to fly it. Run them from the top folder of the repo.

The everyday check is still `python tests/test_sim.py`. These go further, and are what a simulator has to pass before it gets a name (see `NAMES.md`).

| Tool | What it does | Takes |
|---|---|---|
| `stress_test.py` | Runs the real program with no visible window and throws random keys, clicks, resizes and test starts at it. Reports errors, broken numbers and frame times | About 1 minute per 5 simulated minutes |
| `settings_sweep.py` | Changes every setting one at a time and prints what it did to a standard flight | About 2 minutes |
| `settings_followup.py` | Checks the settings the sweep can't see (gusts, filters, vibration, battery size, airmode, camera) | Under a minute |
| `live_window_test.py` | Opens the real window and flies it with real key presses: every key, then all eight flight tests. Windows only. Don't touch the keyboard or mouse while it runs | About 5 minutes |
| `frame_time.py` | Times the physics and the drawing per frame, in the window and at full screen size | Seconds |
| `check_docs.py` | Checks that every command, file and setting named in the docs is real | Seconds |


```
python tools/stress_test.py 7 10 --set video_loss=60     # seed 7, 10 simulated minutes, with a setting
python tools/settings_sweep.py mass_g wind                # only these settings
python tools/live_window_test.py free                     # only the free-flight checks
python tools/live_window_test.py 1,7,6b                   # only those flight tests
```

The pilots in the stress and live tests read the simulator's true position, the way a person's eyes would. They are test rigs, not something the real drone could do.

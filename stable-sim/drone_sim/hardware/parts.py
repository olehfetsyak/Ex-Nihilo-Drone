# The build: every number that describes the physical drone and its firmware settings.
#
# Defaults match the parts we're planning: ~55 g 1S brushless quad, ESP32-C3 flight controller
# running an ESP-FC-style flight stack, ESP-NOW link, XIAO ESP32-S3 Sense camera.
#
# Change a default here once a part is picked or measured. To try a value without editing this
# file, pass it on the command line:  python run_sim.py --set mass_g=58
#
# To add a setting: give it a default in PARTS and a range in LIMITS, then use it in drone.py as
# p["its_name"]. The --set and --list commands pick it up by themselves.

# Flight controller chips. loop_cost_us is how long one pass of the flight loop takes;
# radio_share is the part of the chip's time the radio takes on a single-core chip.
CHIPS = {
    "c3": {"name": "ESP32-C3 (no FPU, 1 core)", "loop_cost_us": 420, "radio_share": 0.18},
    "s3": {"name": "ESP32-S3 (FPU, 2 cores)", "loop_cost_us": 110, "radio_share": 0.0},
    "esp32": {"name": "ESP32 (FPU, 2 cores)", "loop_cost_us": 130, "radio_share": 0.0},
}

# 3.3 V regulators. dropout is how far the output sits under the battery voltage;
# min_in is for a buck-boost, which holds 3.3 V until the battery falls under it.
REGULATORS = {
    "ams1117": {"name": "AMS1117 (about 1.1 V dropout)", "dropout": 1.1, "min_in": 0},
    "ldo": {"name": "Low-dropout LDO (about 0.2 V)", "dropout": 0.2, "min_in": 0},
    "buckboost": {"name": "Buck-boost (holds 3.3 V down to 2.5 V in)", "dropout": -9, "min_in": 2.5},
}

# Settings that are a choice from a list instead of a number.
CHOICES = {"chip": list(CHIPS), "regulator": list(REGULATORS), "failsafe": ["land", "drop"]}

PARTS = {
    # frame and motors
    "mass_g": 55,             # all-up weight with battery
    "motor_thrust_g": 40,     # most thrust one motor gives, on a full battery
    "arm_mm": 40,             # centre of the frame to a motor
    "cg_offset_mm": 0,        # centre of gravity off to the right of the middle
    "motor_tau_ms": 25,       # how long a motor takes to spin up or down
    "esc_delay_ms": 6,        # delay through the 4-in-1 ESC
    "yaw_coef": 0.006,        # how much turning force a motor makes per unit of thrust
    "motor_mismatch": 3,      # % difference between motors
    "level_error_deg": 1.0,   # how far off level the flight controller board sits
    # air drag: a guess until the real drone is timed over a known distance (see README.md in this folder)
    "drag_linear": 0.04,      # newtons of drag per m/s of airspeed
    "drag_area_cm2": 120,     # how big a flat plate the drone looks like to the air. Probably too high
    # power
    "battery_mah": 300,
    "rint_mohm": 45,          # battery internal resistance: sets how far the voltage sags under load
    "start_charge": 100,      # %
    "regulator": "ldo",
    "brownout_v": 2.9,        # the flight controller resets if its supply falls under this
    # flight controller
    "chip": "c3",
    "loop_hz": 1000,          # flight loop rate asked for; a slow chip may not reach it
    "gyro_lpf": 90, "dterm_lpf": 70,   # filter cutoffs in Hz
    "acc_trust": 0.15,        # how hard the accelerometer pulls the attitude estimate
    "gyro_cal": True,         # calibrate the gyro at power-up
    "airmode": True,
    # tuning (see tuning.py and  python run_sim.py --tune)
    "max_angle": 45,          # degrees of lean at full stick
    "angle_gain": 5,          # how hard it chases the lean angle asked for
    "max_rate": 600, "yaw_rate": 300,   # deg/s
    "p_roll": 45, "i_roll": 80, "d_roll": 60,   # roll and pitch share these
    "p_yaw": 45, "i_yaw": 80,
    # radio link (laptop <-> ESP-NOW bridge <-> drone)
    "packet_hz": 100,         # commands up
    "telem_hz": 50,           # flight data down
    "link_delay_ms": 8,
    "link_loss": 0,           # % of packets lost
    # failsafe (runs on the drone when the link is lost)
    "failsafe_delay_ms": 300,
    "failsafe": "land",       # land: level and a fixed throttle. drop: motors off
    "failsafe_throttle": 0.55,
    "failsafe_land_s": 6,     # motors off after this long in failsafe
    # sensors (LSM6DSM)
    "gyro_noise": 1.0, "gyro_bias": 0.5, "acc_noise": 0.03,
    "vibration": 0.3, "vib_freq": 280,   # motor vibration reaching the gyro, and its frequency in Hz
    # XIAO camera
    "cam_tilt": 20,           # degrees up from straight ahead
    "cam_fov": 66,
    "video_delay_ms": 120,
    "video_fps": 20,
    "video_loss": 0,          # % of frames that never arrive, so the picture stutters and freezes
}

# Lowest and highest value each number setting may take. Outside these the simulator either
# divides by zero or stops meaning anything (for example a video too slow to ever count as working).
LIMITS = {
    "mass_g": (10, 500), "motor_thrust_g": (5, 500), "arm_mm": (10, 300), "cg_offset_mm": (-20, 20),
    "motor_tau_ms": (1, 500), "esc_delay_ms": (0, 100), "yaw_coef": (0, 0.1),
    "motor_mismatch": (0, 50), "level_error_deg": (0, 20), "drag_linear": (0, 1), "drag_area_cm2": (0, 2000),
    "battery_mah": (10, 10000), "rint_mohm": (0, 1000), "start_charge": (1, 100), "brownout_v": (2, 3.3),
    "loop_hz": (50, 8000), "gyro_lpf": (1, 1000), "dterm_lpf": (1, 1000), "acc_trust": (0, 5),
    "max_angle": (5, 80), "angle_gain": (0.5, 20), "max_rate": (10, 2000), "yaw_rate": (10, 2000),
    "p_roll": (0, 500), "i_roll": (0, 500), "d_roll": (0, 500), "p_yaw": (0, 500), "i_yaw": (0, 500),
    "packet_hz": (5, 1000), "telem_hz": (5, 1000),   # slower than this and the link always counts as lost
    "link_delay_ms": (0, 1000), "link_loss": (0, 100),
    "failsafe_delay_ms": (50, 5000), "failsafe_throttle": (0, 1), "failsafe_land_s": (0, 60),
    "gyro_noise": (0, 100), "gyro_bias": (0, 50), "acc_noise": (0, 2), "vibration": (0, 20), "vib_freq": (1, 2000),
    "cam_tilt": (-90, 90), "cam_fov": (20, 150),
    "video_delay_ms": (0, 900),   # the video link only remembers the last second
    "video_fps": (3, 120),        # slower than this and the video always counts as lost
    "video_loss": (0, 100),
}

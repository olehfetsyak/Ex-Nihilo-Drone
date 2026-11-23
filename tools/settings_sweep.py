# Settings sweep: changes every setting one at a time, flies a standard routine (take off, hover,
# lean, turn, video loss, link loss) and prints what changed compared with the default drone.
# "NO MEASURED CHANGE" means this routine can't see that setting; settings_followup.py covers those.
#
#     python tools/settings_sweep.py                 every setting
#     python tools/settings_sweep.py mass_g wind     only these
import math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
sys.path.insert(0, ROOT)
from drone_sim.command_center import CommandCenter
from drone_sim.params import default_params
from drone_sim.quat import euler
from drone_sim.hardware.drone import Sim, R2D

F = 1 / 60


def measure(**params):
    sim = Sim(params); cc = CommandCenter(sim); m = {}

    def fly(held, secs, each=None):
        for _ in range(round(secs / F)):
            cc.update(set(held), F); cc.advance(F)
            if each: each()

    fly([], 0.7)
    cc.arm = True
    fly([], 0.4)
    m["armed"] = sim.armed
    m["idle_lo"], m["idle_hi"] = min(sim.cmd), max(sim.cmd)
    m["load"] = sim.loop_load
    m["tel_age"] = sim.t - sim.telem_t
    cc.pilot.key_down("w"); fly(["w"], 1.0)
    wob = []
    fly([], 4.0, lambda: wob.append(math.hypot(sim.w[0], sim.w[1]) * R2D))
    m["z"], m["vz"], m["thr"], m["vbat"], m["soc"] = sim.pos[2], sim.vel[2], sim.rc["thr"], sim.vload, sim.soc
    m["x"], m["y"] = sim.pos[0], sim.pos[1]
    m["wobble"] = sum(wob[-120:]) / 120
    m["yawdrift"] = euler(sim.q)[2] * R2D
    m["brownout"] = any("Brownout" in e["msg"] for e in sim.events)
    m["vid_lag"] = sim.t - cc.video.frame[0] if cc.video.frame else -1
    m["vid_ok"] = cc.safety.video_ok
    m["link_ok"] = cc.safety.link_ok
    m["crashed"] = sim.crashed
    # lean and yaw
    cc.pilot.key_down("right"); fly(["right"], 0.25); m["roll_early"] = euler(sim.q)[0] * R2D
    fly(["right"], 0.75); m["roll"] = euler(sim.q)[0] * R2D
    fly([], 1.5)
    y0 = euler(sim.q)[2]; cc.pilot.key_down("d"); fly(["d"], 0.5)
    m["yawrate"] = -(euler(sim.q)[2] - y0) * R2D / 0.5
    fly([], 1.0)
    # video loss: time until the landing starts
    cc.video.cut = True; t0 = sim.t; m["land_after"] = -1
    def w1():
        if m["land_after"] < 0 and cc.safety.landing: m["land_after"] = sim.t - t0
    fly([], 7.5, w1)
    cc.video.cut = False; cc.take_control() ; cc.pilot.key_down("w"); fly(["w"], 2.5); fly([], 1.5)
    # link loss: time to failsafe, sink, time to motor cut
    z0 = sim.pos[2]; sim.link_cut = True; t0 = sim.t; m["fs_after"] = m["cut_after"] = -1
    def w2():
        if m["fs_after"] < 0 and sim.failsafe: m["fs_after"] = sim.t - t0
        if m["cut_after"] < 0 and not sim.armed: m["cut_after"] = sim.t - t0
    fly([], 1.5, w2); m["fs_sink"] = z0 - sim.pos[2]
    fly([], 8, w2)
    return m


CHANGES = {
    "mass_g": 65, "motor_thrust_g": 50, "arm_mm": 80, "cg_offset_mm": 3, "motor_tau_ms": 80, "esc_delay_ms": 30,
    "yaw_coef": 0.002, "motor_mismatch": 12, "level_error_deg": 5.0, "drag_linear": 0.01, "drag_area_cm2": 40,
    "battery_mah": 150, "rint_mohm": 150,
    "start_charge": 40, "regulator": "ams1117", "brownout_v": 3.25, "chip": "s3", "loop_hz": 4000, "gyro_lpf": 20,
    "dterm_lpf": 15, "acc_trust": 0.0, "gyro_cal": False, "max_angle": 20, "angle_gain": 2, "max_rate": 60,
    "yaw_rate": 100, "airmode": False, "p_roll": 15, "i_roll": 0, "d_roll": 0, "p_yaw": 10, "i_yaw": 0,
    "packet_hz": 10, "telem_hz": 5, "link_delay_ms": 150, "link_loss": 70, "failsafe_delay_ms": 1000,
    "failsafe": "drop", "failsafe_throttle": 0.3, "failsafe_land_s": 2, "gyro_noise": 8.0, "gyro_bias": 5.0,
    "vibration": 3.0, "vib_freq": 60, "acc_noise": 0.3, "wind": 2.0, "gust": 1.0, "wind_dir": 0,
    "cam_tilt": 45, "cam_fov": 100, "video_delay_ms": 600, "video_fps": 5, "video_loss": 60, "kb_strength": 1.0,
    "hover_throttle": 0.7, "safety_max_tilt": 10.0, "safety_hover_s": 2.0,
}
missing = set(default_params()) - set(CHANGES)
print("settings not in the sweep:", missing or "none")
base = measure()
print("BASE", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in base.items()})
for name, value in CHANGES.items():
    if len(sys.argv) > 1 and name not in sys.argv[1:]:
        continue
    kw = {name: value}
    if name == "wind_dir": kw["wind"] = 2.0
    if name == "gust": kw["wind"] = 0.0
    try:
        m = measure(**kw)
    except Exception as e:
        print(f"{name}={value}: ERROR {type(e).__name__}: {e}"); continue
    ref = measure(wind=2.0) if name == "wind_dir" else base
    diff = []
    for k, v in m.items():
        b = ref[k]
        if isinstance(v, bool) or isinstance(b, bool):
            if v != b: diff.append(f"{k} {b}->{v}")
        elif abs(v - b) > max(0.05, 0.1 * abs(b)):
            diff.append(f"{k} {b:.2f}->{v:.2f}")
    print(f"{name}={value}: " + ("; ".join(diff) or "NO MEASURED CHANGE"), flush=True)

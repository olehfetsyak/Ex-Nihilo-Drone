# Tuning report: flies one build through a fixed routine, without a window, and measures how it
# responds. Use it to compare tuning values before trying them on the real drone:
#
#     python run_sim.py --tune --set p_roll=60 --set d_roll=40
#
# The routine: take off, hold 2 m, then a sudden half-stick roll and a sudden half-stick turn.
# A simple height hold flies the throttle using the simulator's true height, which the real drone
# doesn't have. That's fine here: it is the test rig, not the drone.
#
# The numbers are only as good as the build settings. Put in the measured weight and thrust first.
import math

from ..quat import euler
from .drone import PHYSICS_HZ, R2D, Sim

HEIGHT = 2.0


# Flies the routine and returns the measurements, or {"failed": why}.
def measure(settings=None):
    sim = Sim(settings)
    p, state = sim.p, {"base": 0.5}

    # Run for a while, holding height. sample() is called every millisecond.
    def run(seconds, sample=None, roll=0.0, yaw=0.0):
        for i in range(round(seconds * PHYSICS_HZ)):
            if i % 40 == 0:   # height hold, 100 times a second
                err = HEIGHT - sim.pos[2]
                state["base"] = max(0.1, min(0.95, state["base"] + 0.15 * err * 0.01))
                thr = max(0.0, min(1.0, state["base"] + 0.35 * err - 0.3 * sim.vel[2]))
                sim.rc.update(roll=roll, pitch=0.0, yaw=yaw, thr=thr, arm=True)
            sim.step()
            if sample and i % 4 == 0:
                sample()

    sim.run(0.7)                 # gyro calibration
    sim.rc.update(arm=True)
    sim.run(0.4)
    if not sim.armed:
        return {"failed": "it would not arm"}
    run(6.0)
    why = ("the flight controller browned out" if any("Brownout" in e["msg"] for e in sim.events) else
           sim.crash_msg if sim.crashed else
           "" if abs(sim.pos[2] - HEIGHT) < 0.5 else
           "it shakes too hard to hold its height (P or D far too high, or too much delay)"
           if max(sim.cmd) - min(sim.cmd) > 0.3 else "it could not lift off or hold 2 m")
    if why:
        return {"failed": why}

    # hover: throttle needed, how much it shakes, how hard the motors work against each other
    thr, wobble, spread = [], [], []

    def hover_sample():
        thr.append(sim.rc["thr"])
        wobble.append(math.hypot(sim.w[0], sim.w[1]) * R2D)
        spread.append(max(sim.cmd) - min(sim.cmd))

    run(2.0, hover_sample)
    mean = lambda v: sum(v) / len(v)
    out = {"hover_thr": mean(thr), "vbat": sim.vload, "wobble": mean(wobble), "spread": mean(spread),
           "load": sim.loop_load, "loop_hz": 1 / max(1 / p["loop_hz"], sim.loop_load / p["loop_hz"])}
    # the hover_throttle setting is this throttle scaled to a 4.0 V battery (see pilot.hover_throttle)
    out["hover_setting"] = ((0.05 + 0.95 * out["hover_thr"]) * sim.vload / 4.0 - 0.05) / 0.95

    # roll step: half stick, held for a second
    angles = []
    run(1.0, lambda: angles.append(euler(sim.q)[0] * R2D), roll=0.5)
    final = mean(angles[-200:])
    out["roll_target"], out["roll_final"] = 0.5 * p["max_angle"], final
    out["roll_rise"] = next((i / 1000 for i, a in enumerate(angles) if a >= 0.9 * final), None) if final > 1 else None
    out["roll_over"] = max(0.0, (max(angles) - final) / final * 100) if final > 1 else 0.0
    run(1.5)

    # yaw step: half stick, held for a second. Stick right turns right, which is a negative yaw rate
    rates = []
    run(1.0, lambda: rates.append(-sim.w[2] * R2D), yaw=0.5)
    target = 0.5 * p["yaw_rate"]
    out["yaw_target"], out["yaw_final"] = target, mean(rates[-200:])
    out["yaw_rise"] = next((i / 1000 for i, r in enumerate(rates) if r >= 0.9 * target), None)
    if sim.crashed:
        return {"failed": sim.crash_msg + " during the routine"}
    return out


# What the numbers mean, in words.
def advice(m):
    notes = []
    if m["wobble"] > 5:
        notes.append("It shakes in the hover: P or D is too high, or there is too much delay (ESC, filters, slow loop).")
    elif m["spread"] > 0.15:
        notes.append("The motors fight each other in the hover: vibration or noise is getting through. They will run hot.")
    if m["roll_rise"] is None:
        notes.append("It never reached the lean it was asked for.")
    elif m["roll_over"] > 15:
        notes.append("It overshoots the lean: more D, or less angle_gain. Very low P does this too.")
    elif m["roll_rise"] > 0.6:
        notes.append("It is slow to lean: raise angle_gain (or P if it feels loose).")
    if m["yaw_rise"] is None:
        notes.append("It never reached the turn rate asked for: yaw is weak on a small quad, so lower yaw_rate or raise p_yaw.")
    if m["load"] > 1:
        notes.append("The chip can't keep up with loop_hz: lower it, or use a faster chip.")
    if abs(m["roll_final"] - m["roll_target"]) > 3:
        notes.append("It settles at the wrong lean angle: check level_error_deg and i_roll.")
    return notes or ["Nothing stands out."]


def _column(m):
    if "failed" in m:
        return None
    ms = lambda v: "never" if v is None else f"{v * 1000:.0f} ms"
    return [f"{m['hover_thr'] * 100:.0f}% at {m['vbat']:.2f} V", f"{m['hover_setting']:.2f}",
            f"{m['wobble']:.1f} deg/s", f"{m['spread'] * 100:.0f}%",
            f"{m['roll_final']:.1f} of {m['roll_target']:.1f} deg", ms(m["roll_rise"]), f"{m['roll_over']:.0f}%",
            f"{m['yaw_final']:.0f} of {m['yaw_target']:.0f} deg/s", ms(m["yaw_rise"]),
            f"{m['load'] * 100:.0f}%, runs at {m['loop_hz']:.0f} Hz"]


ROWS = ["hover throttle", "  as the hover_throttle setting", "hover wobble", "motor spread in the hover",
        "roll step: lean after 1 s", "  time to 90% of it", "  overshoot",
        "yaw step: turn rate after 1 s", "  time to 90% of the target", "flight loop load"]


# Prints the report for this build next to the default build.
def report(settings):
    mine = measure(settings)
    if "failed" in mine:
        print(f"This build did not finish the routine: {mine['failed']}.")
        return mine
    cols = [("this build", _column(mine))]
    if settings:
        cols.append(("default build", _column(measure())))
    print(f"{'':32s}" + "".join(f"{name:28s}" for name, _ in cols))
    for i, row in enumerate(ROWS):
        print(f"{row:32s}" + "".join(f"{col[i]:28s}" for _, col in cols))
    print()
    for note in advice(mine):
        print(note)
    print("\nThese numbers are only as good as the build settings: measure the real weight and thrust first.")
    return mine

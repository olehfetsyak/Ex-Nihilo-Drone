# Hardware flight tests, on keys 1 to 4: what the build has to cope with (weight, battery and
# regulator, a crooked build, wind).
#
# To add one:
#   1. write a check function like the ones below. It is called once per frame with
#        run   the test in progress (see FlightTests in ../flight_tests.py)
#        sim   the drone, with its true height, position and speed
#        dt    seconds since the last frame
#      and returns None while the test is still going, or (passed, message) when it is over.
#      Set run.note to tell the pilot what to do next
#   2. add it to HARDWARE_TESTS with the next free number. "settings" are laid over the build for
#      the length of the test, and "rings" are drawn as (x, y, height, radius)
#
# Helpers a check can use:
#   run.holding(why, dt)   counts 10 s of every condition being met. Pass why="" when they all
#                          are, or a few words saying what is missing. True once the 10 s are up
#   run.landing(sim)       None until the drone is down and disarmed, then pass or fail on how
#                          hard it touched down
import math

HOVER_SPEED = 0.8  # m/s: moving slower than this counts as hovering


def speed(sim):
    return math.sqrt(sum(v * v for v in sim.vel))


def hover(run, sim, dt):
    r = run.test["rings"][0][3]
    why = ("get between the two rings" if not 0.5 < sim.pos[2] < 2 else
           "come back over the pad" if math.hypot(sim.pos[0], sim.pos[1]) > r else
           f"too fast to call a hover ({speed(sim):.1f} m/s, needs under {HOVER_SPEED})" if speed(sim) > HOVER_SPEED else "")
    run.total = run.total + sim.rc["thr"] * dt if not why else 0.0
    if run.holding(why, dt):
        return True, (f"This build hovers at about {run.total / run.hold * 100:.0f}% throttle "
                      f"({sim.telem['vbat']:.2f} V, trim {round(run.cc.pilot.trim * 100):+d}%)")


def weak_battery(run, sim, dt):
    if not sim.fc_on:
        return False, f"Brownout: the regulator only gave the flight controller {sim.vreg:.2f} V"
    if run.stage == 0:
        run.note = "climb to the ring at 3 m"
        if sim.pos[2] > 3:
            run.stage = 1
        return None
    return run.landing(sim)


def stay_close(run, sim, dt):
    r = run.test["rings"][0][3]
    why = ("take off and get above 0.5 m" if sim.pos[2] < 0.5 else
           "outside the ring: come back" if math.hypot(sim.pos[0], sim.pos[1]) > r else "")
    if run.holding(why, dt):
        return True, f"Stayed inside {r} m for 10 s"


HARDWARE_TESTS = {
    1: {"who": "hardware", "name": "Hover", "check": hover, "settings": {},
        "brief": "Hover between the two rings, nearly still, for 10 s. Q/E trims the hover.",
        "rings": [(0, 0, 0.5, 1.5), (0, 0, 2, 1.5)]},
    2: {"who": "hardware", "name": "Weak battery", "check": weak_battery, "settings": {"start_charge": 25},
        "brief": "Battery at 25%. Climb to 3 m, then land softly and disarm, without a brownout.",
        "rings": [(0, 0, 3, 1)]},
    3: {"who": "hardware", "name": "Crooked build", "check": stay_close,
        "settings": {"level_error_deg": 3.0, "motor_mismatch": 8},
        "brief": "Board 3 degrees off level, motors 8% apart. Stay above 0.5 m inside the ring for 10 s.",
        "rings": [(0, 0, 0, 2)]},
    4: {"who": "hardware", "name": "Breeze", "check": stay_close, "settings": {"wind": 2.0, "gust": 0.5},
        "brief": "A 2 m/s breeze with gusts. Stay above 0.5 m inside the ring for 10 s.",
        "rings": [(0, 0, 0, 3)]},
}

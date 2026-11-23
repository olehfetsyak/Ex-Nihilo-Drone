# Flight tests: short challenges you fly in the simulator, started with the number keys.
#
#   1-4  hardware: what the build has to cope with. These live in hardware/flight_tests.py
#   5-8  software: what the laptop side has to cope with (link drops, video drops, lag, lost packets)
#
# Each test changes a few settings on top of the ones the simulator was started with, shows rings to
# fly to, and says PASS or FAIL. So the same test can be flown against a different build:
#
#     python run_sim.py --set regulator=ams1117     then press 2
#
# The judging uses the simulator's true height and position. That's fine: it's the examiner, not
# the drone or the laptop.
import math

from .hardware.flight_tests import HARDWARE_TESTS

SOFT = 1.8         # m/s: a touchdown slower than this counts as a soft landing (holding S comes down at 1.5)


# ---------------------------------------------------------------- software
def link_drop(run, sim, dt):
    if run.stage == 0:
        run.note = "climb to the ring at 4 m" if sim.pos[2] < 4 else "above 4 m: let go and let it settle"
        if sim.pos[2] > 4 and abs(sim.vel[2]) < 0.5:   # a steady hover, so the drop is the failsafe's doing
            sim.link_cut, run.stage, run.t0 = True, 1, sim.t
            sim.event("Link cut for 3 s", "bad")
    elif run.stage == 1:
        run.note = "link down: the drone's own failsafe is flying"
        if sim.t - run.t0 > 3:
            sim.link_cut, run.stage = False, 2
            sim.event("Link restored", "good")
    else:
        return run.landing(sim)


def video_drop(run, sim, dt):
    video = run.cc.video
    if run.stage == 0:
        fast = math.hypot(sim.vel[0], sim.vel[1]) > 2
        run.note = "fly forward over the ring at more than 2 m/s" if sim.pos[0] < 5 or fast else "too slow: keep going forward, faster"
        if sim.pos[0] > 5 and sim.pos[2] > 0.5 and fast:
            video.cut, run.stage = True, 1
            sim.event("Video cut", "bad")
        return None
    out = run.landing(sim)
    if not out:
        run.note = "video is gone for good: take control (T) or let it land. Then disarm"
    return out


def land_in_ring(run, sim, dt):
    x, y, _, r = run.test["rings"][0]
    out = run.landing(sim)
    if out and out[0]:
        d = math.hypot(sim.pos[0] - x, sim.pos[1] - y)
        if d > r:
            return False, f"Landed {d:.1f} m from the middle of the ring (it is {r} m to the edge)"
        return True, f"Landed {d:.1f} m from the middle, at {run.touch:.1f} m/s"
    return out


# rings are (x, y, height, radius)
TESTS = {
    **HARDWARE_TESTS,   # 1 to 4, in hardware/flight_tests.py
    5: {"who": "software", "name": "Link drop", "check": link_drop, "settings": {},
        "brief": "Hover above 4 m. The link drops for 3 s. Catch it when it comes back, land softly, disarm.",
        "rings": [(0, 0, 4, 1)]},
    6: {"who": "software", "name": "Video drop", "check": video_drop, "settings": {},
        "brief": "Fly forward over the ring at speed. The video drops. Get it down softly and disarm.",
        "rings": [(5, 0, 0, 1)]},
    7: {"who": "software", "name": "Lag", "check": land_in_ring,
        "settings": {"link_delay_ms": 60, "video_delay_ms": 350},
        "brief": "Commands 60 ms late, video 350 ms late. Land in the ring and disarm. Try camera C.",
        "rings": [(8, -3, 0, 1.5)]},
    8: {"who": "software", "name": "Lost packets", "check": land_in_ring, "settings": {"link_loss": 85},
        "brief": "85% of packets are lost both ways. Land in the ring and disarm.",
        "rings": [(-6, 3, 0, 1.5)]},
}


class FlightTests:
    def __init__(self, cc):
        self.cc, self.base = cc, dict(cc.sim.p)   # the settings the simulator was started with
        self.start(0)

    # Start test n, or go back to free flight with 0. Starting the same one again is a retry.
    def start(self, n):
        sim = self.cc.sim
        self.n, self.test = n, TESTS.get(n)
        sim.p.clear()
        sim.p.update(self.base)
        if self.test:
            sim.p.update(self.test["settings"])
            sim.rng.seed(1000 + n)   # the same build and weather every time, so a retry is fair
        else:
            sim.rng.seed(sim.seed)   # free flight: a reset gives back the same drone
        self.cc.reset()
        self.hold = self.total = self.sink = self.t0 = 0.0
        self.stage, self.flown, self.touch = 0, False, None
        self.note, self.result = "", None   # result becomes (passed, message)

    def rings(self):
        return self.test["rings"] if self.test else []

    # Once per frame, after the physics.
    def update(self, dt):
        sim = self.cc.sim
        if not self.test or self.result:
            return
        if sim.crashed:
            self.result = (False, sim.crash_msg)
            return
        if sim.pos[2] > 0.5:
            self.flown, self.touch = True, None
        if not sim.on_ground:
            self.sink = -sim.vel[2]   # so at touchdown this is how fast it came down
        self.result = self.test["check"](self, sim, dt)

    # Counts how long the conditions have all been met without a break. why says what is missing
    # ("" when nothing is), and the clock starts again from zero whenever something is. True after 10 s.
    def holding(self, why, dt, seconds=10):
        self.hold = self.hold + dt if not why else 0.0
        self.note = why or f"holding: {self.hold:.0f} of {seconds} s"
        return self.hold >= seconds

    # None while still flying, then pass or fail once the drone is down and disarmed.
    def landing(self, sim):
        self.note = "land softly, then disarm"
        if not (self.flown and sim.on_ground):
            return None
        if self.touch is None:
            self.touch = self.sink
        if self.touch > SOFT:
            return False, f"Came down at {self.touch:.1f} m/s. A soft landing is under {SOFT} m/s"
        if sim.armed:
            self.note = "down. Now disarm (Space)"
            return None
        return True, f"Landed at {self.touch:.1f} m/s"

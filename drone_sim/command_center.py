# Command Center: the laptop side, wired the same way as the software plan.
#
#     keyboard -> pilot -> Safety Layer -> Radio Bridge (sim.rc) -> drone
#     drone -> flight data (sim.telem) and video -> back here
#
# Nothing in here reads the simulator's true state (position, speed, height): the real laptop won't
# have it either.
#
# No graphics, so the tests fly through exactly the same code as the window does.
from .pilot import KeyboardPilot
from .safety import SafetyLayer
from .hardware.drone import PHYSICS_HZ
from .video import VideoLink


class CommandCenter:
    def __init__(self, sim):
        self.sim = sim
        self.pilot, self.video, self.safety = KeyboardPilot(), VideoLink(), SafetyLayer(sim.p)
        self.arm = False

    def reset(self):
        self.sim.reset()
        self.pilot.reset()
        self.video.reset()
        self.safety.reset()
        self.arm = False

    def disarm(self):
        self.arm = False

    # The arm key. It only ever arms: disarming has its own key and the DISARM button, so a second
    # press can't stop the motors by mistake. Refused unless the laptop has both the link and the video.
    def arm_drone(self):
        if self.arm:
            return
        if self.safety.link_ok and self.safety.video_ok:
            self.arm = True
        else:
            self.sim.event("Can't arm: no " + ("video" if self.safety.link_ok else "link"), "bad")

    # The take-control key: fly on the keys even though the video is gone.
    def take_control(self):
        if self.safety.take_control():
            self.sim.event("Pilot took control" + ("" if self.safety.video_ok else " without video"))

    # Once per screen frame: read the keys, run the safety layer, hand the result to the bridge.
    def update(self, held, dt):
        sim, tel = self.sim, self.sim.telem
        self.video.update(sim)
        if not self.safety.check(sim.t - sim.telem_t, self.video.age(sim), dt):
            held = set()   # link or video lost: the flight keys are locked out
        cmd = self.pilot.update(held, tel, sim.p, dt)
        cmd = self.safety.update(cmd, self.pilot.active, tel, self.pilot.trim)
        sim.rc.update(cmd, arm=self.arm)

    # Run the drone forward by dt, saving a camera pose every 5 ms for the video link.
    def advance(self, dt):
        for i in range(round(dt * PHYSICS_HZ)):
            self.sim.step()
            if i % 20 == 0:
                self.video.record(self.sim)

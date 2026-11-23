# Video link from the XIAO camera to the laptop: delayed, low frame rate, and it can stutter or
# drop out.
#
# Keeps a short history of where the camera was, and hands back the newest frame the laptop would
# have received by now.
import math
import random
from collections import deque


class VideoLink:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hist = deque()   # (time, position, attitude)
        self.frame = None     # newest frame on the laptop
        self.slot = None      # the camera frame time last looked at
        self.cut = False
        self.rng = random.Random(1)   # its own dice, so lost frames don't change how the drone flies

    def record(self, sim):
        self.hist.append((sim.t, list(sim.pos), list(sim.q)))
        while self.hist[0][0] < sim.t - 1:
            self.hist.popleft()

    def update(self, sim):
        if self.cut:
            return
        per = 1 / sim.p["video_fps"]
        t_want = math.floor((sim.t - sim.p["video_delay_ms"] / 1000) / per) * per
        if t_want == self.slot:
            return
        self.slot = t_want   # a new frame is due: it either arrives or is lost for good
        if self.rng.random() * 100 < sim.p["video_loss"]:
            return
        for f in reversed(self.hist):
            if f[0] <= t_want:
                self.frame = f
                break

    # Seconds since the newest frame arrived on the laptop. The video delay doesn't count: a late
    # picture that keeps coming is still a working video.
    def age(self, sim):
        if self.cut and self.frame is None:
            return 9.0
        since = self.frame[0] if self.frame else 0.0   # before the first frame, count from power-up
        return sim.t - since - sim.p["video_delay_ms"] / 1000

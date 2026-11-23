# Safety Layer: sits between the Command Center and the Radio Bridge.
#
# Every command goes through here before it reaches the bridge:
#
#   - tilt limit. The drone has no speed sensor, so limiting the lean is also how speed is limited
#   - video lost -> the flight keys are locked out and the drone hovers for safety_hover_s seconds
#     while the screen asks whether to take control (T). Nobody takes it -> it lands. T hands the
#     keys back at any point, so the pilot always overrides. DISARM and the Q/E trim always work
#   - link lost -> send a calm hover command; the drone's own failsafe is flying until the link is back
#   - no arming without both the link and the video
#
# It only uses the flight data the drone reports back (tel). With no height or speed to go on,
# "hover" means level sticks and the hover throttle, and "land" means a little less throttle than
# that. The drone drifts while it does either, and someone still has to disarm once it's down.
import math

from .pilot import hover_throttle

LINK_TIMEOUT = 0.3    # s without flight data before the link counts as lost
VIDEO_TIMEOUT = 0.5   # s without a new frame before the video counts as lost
LAND_THROTTLE = 0.03  # how far under hover throttle the landing goes
LOW_BATTERY = 3.4     # V under load: time to land


class SafetyLayer:
    def __init__(self, p):
        self.p = p
        self.reset()

    def reset(self):
        self.link_ok = self.video_ok = True
        self.source = "keyboard"   # who is flying: "keyboard" or "safety"
        self.reason = ""           # why safety took over
        self.lost_t = 0.0          # how long the video has been gone
        self.takeover = False      # the pilot chose to fly without video
        self.landing = False       # safety is bringing the drone down
        self.low_battery = False

    # Called first each frame. False means the flight keys are locked out.
    def check(self, link_age, video_age, dt):
        self.link_ok, self.video_ok = link_age < LINK_TIMEOUT, video_age < VIDEO_TIMEOUT
        if self.video_ok:
            self.lost_t, self.takeover = 0.0, False
        else:
            self.lost_t += dt
        return self.link_ok and (self.video_ok or self.takeover)

    # The pilot's answer to "take control?". True if there was something to take over.
    def take_control(self):
        if self.video_ok and not self.landing:
            return False
        self.takeover, self.landing = not self.video_ok, False
        return True

    # Seconds of hovering left before it lands by itself.
    def countdown(self):
        return max(0.0, self.p["safety_hover_s"] - self.lost_t)

    # Takes the pilot's command and returns the one that is safe to send.
    def update(self, cmd, keys_active, tel, trim=0.0):
        self.source, self.reason = "keyboard", ""
        self.low_battery = self.low_battery or (tel["armed"] and tel["vbat"] < LOW_BATTERY)
        flying = cmd["thr"] > 0   # safety never starts the motors lifting on its own
        hover = hover_throttle(self.p, tel, trim)
        if not self.link_ok:
            self.source, self.reason = "safety", "link lost"
            return self._level(hover if flying else 0.0)
        unseen = not self.video_ok and not self.takeover
        # a landing, once started, carries on even after the video comes back: we can't tell whether
        # the drone is already down, and handing back hover throttle would lift it off again. The
        # first flight key pressed after the video is back takes over.
        if not flying or not tel["armed"] or keys_active:
            self.landing = False
        elif unseen and self.countdown() <= 0:
            self.landing = True
        if flying and (unseen or self.landing):
            self.source, self.reason = "safety", "video lost"
            return self._level(hover - (LAND_THROTTLE if self.landing else 0.0))
        # tilt: full stick asks for max_angle, so cap the stick
        roll, pitch = cmd["roll"], cmd["pitch"]
        lim, mag = min(1.0, self.p["safety_max_tilt"] / self.p["max_angle"]), math.hypot(roll, pitch)
        if mag > lim:
            roll, pitch = roll * lim / mag, pitch * lim / mag
        return {"roll": roll, "pitch": pitch, "yaw": cmd["yaw"], "thr": cmd["thr"]}

    def _level(self, thr):
        return {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thr": thr}

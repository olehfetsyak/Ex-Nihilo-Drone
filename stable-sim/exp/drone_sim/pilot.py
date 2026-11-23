# Keyboard pilot: turns held keys into stick values, the way the laptop Command Center sends them.
#
# No graphics library here: pass in the set of held key names, so this is easy to reuse in the real
# Command Center. Key names: "w", "s", "a", "d", "up", "down", "left", "right".
#
#   hands off  -> hover throttle
#   hold W     -> climb (the first W after arming takes off)
#   hold S     -> descend. The drone can't tell when it has landed, so disarm once it's down
#   arrows     -> lean (pitch and roll), A/D -> turn (yaw)
#   tap Q / E  -> hover throttle down / up 1% (pass "q" / "e" to key_down)
#
# It only knows what the drone reports back as flight data (tel): lean angles, battery, armed.
import math

FLIGHT_KEYS = {"w", "s", "a", "d", "up", "down", "left", "right"}


def _ramp(v, target, rate, dt):
    return v + max(-rate * dt, min(rate * dt, target - v))


# Throttle that should hover. It starts from one number, hover_throttle, which is a guess until we
# measure it on the real drone. Thrust drops as the battery sags, and leaning tips thrust sideways,
# so ask for a bit more in both cases.
def hover_throttle(p, tel, trim=0.0):
    lean = max(0.5, math.cos(tel["roll"]) * math.cos(tel["pitch"]))
    speed = (0.05 + 0.95 * (p["hover_throttle"] + trim)) * 4.0 / max(3.0, tel["vbat"]) / math.sqrt(lean)
    return max(0.15, min(0.95, (speed - 0.05) / 0.95))  # the mixer maps throttle to 5..100% motor


class KeyboardPilot:
    def __init__(self):
        self.reset()

    def reset(self):
        self.trim = 0.0   # Q/E nudge the hover throttle when the drone sinks or climbs hands off
        self.flying = self.active = False
        self.k_roll = self.k_pitch = self.k_yaw = self.k_thr = 0.0
        self.order = {}   # when each key went down, for "last key wins" when both of a pair are held
        self._seq = 0

    def key_down(self, name):
        self._seq += 1
        self.order[name] = self._seq
        if name in ("q", "e"):
            self.trim = max(-0.3, min(0.3, self.trim + (0.01 if name == "e" else -0.01)))

    # Returns the stick command {"roll", "pitch", "yaw", "thr"} for this frame.
    def update(self, held, tel, p, dt):
        ks = p["kb_strength"]
        self.active = bool(FLIGHT_KEYS & held)

        def pair(pos, neg):
            a, b = pos in held, neg in held
            if a and b:  # both held: the one pressed last wins
                return 1 if self.order.get(pos, 0) > self.order.get(neg, 0) else -1
            return 1 if a else -1 if b else 0

        # keys are on/off, so ease them in like a stick
        self.k_roll = _ramp(self.k_roll, pair("right", "left") * ks, 4, dt)
        self.k_pitch = _ramp(self.k_pitch, pair("up", "down") * ks, 4, dt)
        self.k_yaw = _ramp(self.k_yaw, pair("d", "a") * min(1, ks * 1.15), 5, dt)

        climb = pair("w", "s")
        if not tel["armed"]:
            self.flying, self.k_thr = False, 0.0
        elif climb > 0:
            self.flying = True                                   # first W after arming = take off
        self.k_thr = _ramp(self.k_thr, {1: 0.10, -1: -0.05, 0: 0.0}[climb], 1.2, dt)
        thr = max(0.0, min(1.0, hover_throttle(p, tel, self.trim) + self.k_thr)) if self.flying else 0.0

        return {"roll": self.k_roll, "pitch": self.k_pitch, "yaw": self.k_yaw, "thr": thr}

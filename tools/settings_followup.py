# Follow-up checks for the settings the sweep can't see: gusts, brownout voltage, gyro
# calibration, filters, vibration, battery size, airmode and the camera settings.
#
#     python tools/settings_followup.py
import math, os, sys
os.environ["SDL_VIDEODRIVER"] = "dummy"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
sys.path.insert(0, ROOT)
from drone_sim.command_center import CommandCenter
from drone_sim.quat import euler
from drone_sim.hardware.drone import Sim, R2D

F = 1 / 60


def hover(secs=5, **params):
    sim = Sim(params); cc = CommandCenter(sim); log = []
    def fly(held, s):
        for _ in range(round(s / F)):
            cc.update(set(held), F); cc.advance(F)
            log.append((math.hypot(*sim.wind_v[:2]), sim.wind_v[2], abs(sim.gyro_f[0]), math.hypot(sim.w[0], sim.w[1]) * R2D, min(sim.cmd), max(sim.cmd)))
    fly([], 0.7); cc.arm = True; fly([], 0.4); cc.pilot.key_down("w"); fly(["w"], 1.0); n = len(log); fly([], secs)
    return sim, cc, log[n:]

std = lambda v: (sum((x - sum(v) / len(v)) ** 2 for x in v) / len(v)) ** 0.5
mean = lambda v: sum(v) / len(v)

for g in (0.0, 0.5, 1.0):
    sim, cc, log = hover(20, gust=g)
    print(f"gust={g}: wind speed mean {mean([l[0] for l in log]):.2f} m/s, vertical std {std([l[1] for l in log]):.2f}, drift {math.hypot(sim.pos[0], sim.pos[1]):.1f} m, crashed {sim.crashed}")
sim, cc, log = hover(20, wind=2.0, gust=0.5)
print(f"wind 2 gust 0.5: wind speed {min(l[0] for l in log):.2f} to {max(l[0] for l in log):.2f} m/s")

for kw in ({}, {"brownout_v": 3.25}, {"start_charge": 30}, {"brownout_v": 3.25, "start_charge": 30}):
    sim, cc, log = hover(3, **kw)
    print(kw, "brownout:", any("Brownout" in e["msg"] for e in sim.events), f"vreg {sim.vreg:.2f}")

for kw in ({}, {"gyro_bias": 5.0}, {"gyro_cal": False}, {"gyro_bias": 5.0, "gyro_cal": False}):
    sim, cc, log = hover(5, **kw)
    print(kw, f"yaw after 5 s: {euler(sim.q)[2] * R2D:.1f} deg")

for kw in ({}, {"gyro_lpf": 5}, {"dterm_lpf": 3}, {"vibration": 3.0}, {"vibration": 3.0, "gyro_lpf": 400}, {"gyro_noise": 8.0}):
    sim, cc, log = hover(4, **kw)
    print(kw, f"filtered gyro roll mean {mean([l[2] for l in log]):.2f} deg/s, true wobble {mean([l[3] for l in log]):.2f} deg/s, motor spread {mean([l[5] - l[4] for l in log]):.3f}")

for kw in ({}, {"battery_mah": 150}, {"battery_mah": 600}):
    sim, cc, log = hover(20, **kw)
    print(kw, f"charge used in 21 s: {(1 - sim.soc) * 100:.1f}%")

# airmode: zero throttle in the air (S can't do this, so set the stick by hand)
for air in (True, False):
    sim = Sim({"airmode": air}); sim.run(0.6); sim.rc.update(arm=True); sim.run(0.3)
    sim.pos[2] = 5.0; sim.q = [math.cos(0.3), math.sin(0.3), 0, 0]; sim.run(0.15)
    print(f"airmode={air}: dropped at zero throttle, tilted 34 deg: motors {min(sim.cmd):.2f} to {max(sim.cmd):.2f}")

# camera settings only change the picture
import pygame
from drone_sim.view import View
pygame.init()
def picture(**kw):
    sim, cc, log = hover(1, **kw)
    v = View(pygame.display.set_mode((400, 300))); v.cam = "fpv"; v.render(sim, F, cc.video)
    return pygame.image.tobytes(v.screen, "RGB")
a = picture()
print("cam_tilt changes the drone camera picture:", picture(cam_tilt=45) != a, " cam_fov:", picture(cam_fov=100) != a, " same settings same picture:", picture() == a)

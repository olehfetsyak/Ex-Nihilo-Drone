# Frame time: how long the physics and the drawing take per frame, in the window and at full
# screen size. A frame has 16.7 ms at 60 frames a second. Opens a real window for a moment.
#
#     python tools/frame_time.py
import os, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
sys.path.insert(0, ROOT)
import pygame
import run_sim
from drone_sim.command_center import CommandCenter
from drone_sim.flight_tests import FlightTests
from drone_sim.hardware.drone import Sim
from drone_sim.view import View
pygame.init()
for loop_hz in (1000, 4000):
    sim = Sim({"loop_hz": loop_hz}); cc = CommandCenter(sim); ft = FlightTests(cc)
    for _ in range(60): cc.update(set(), 1 / 60); cc.advance(1 / 60)
    cc.arm = True; cc.pilot.key_down("w")
    t = time.perf_counter()
    for _ in range(120): cc.update({"w"}, 1 / 60); cc.advance(1 / 60); ft.update(1 / 60)
    print(f"physics for one frame, loop_hz={loop_hz}: {(time.perf_counter() - t) / 120 * 1000:.1f} ms")
for size in ((1100, 700), (2560, 1440)):
    v = View(pygame.display.set_mode(size))
    for cam in ("chase", "fpv", "ground"):
        v.cam = cam; parts = {}
        for name, fn in (("world", lambda: v.render(sim, 1 / 60, cc.video, ft.rings())), ("hud", lambda: v.hud(sim, False, run_sim.KEY_HELP, cc, ft)),
                         ("flip", pygame.display.flip)):
            t = time.perf_counter()
            for _ in range(30): fn()
            parts[name] = (time.perf_counter() - t) / 30 * 1000
        print(f"{size[0]}x{size[1]} {cam:6s}: " + ", ".join(f"{k} {ms:.1f} ms" for k, ms in parts.items()), f"-> total {sum(parts.values()):.1f} ms")
pygame.quit()

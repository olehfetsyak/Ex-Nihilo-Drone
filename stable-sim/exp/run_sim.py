# Drone v.0.0 Sim (Python) by Mori. Stable simulator name: see NAMES.md.
#
# Matches the real drone architecture:
#   Drone: ESP32-C3 flight controller + XIAO ESP32-S3 FPV camera
#   Macbook: keyboard -> Command Center -> Safety Layer -> Radio Bridge (ESP-NOW) -> drone
#
#     python run_sim.py                          fly with the default parts
#     python run_sim.py --set regulator=ams1117  change a setting (repeat --set for more)
#     python run_sim.py --list                   show every setting and its default
#     python run_sim.py --tune --set p_roll=60   print a tuning report for that build, no window
#     python run_sim.py --seed 7                 fly drone number 7 again (each launch is a new one otherwise)
#
# Press H in the window for every key.
import argparse
import random
import sys

from drone_sim import VERSION
from drone_sim.command_center import CommandCenter
from drone_sim.flight_tests import TESTS, FlightTests
from drone_sim.hardware.drone import Sim
from drone_sim.params import CHOICES, LIMITS, default_params
from drone_sim.world import BOXES

KEY_HELP = " | ".join(["Enter arm", "Space DISARM", "W/S up/down", "arrows lean", "A/D turn", "R reset", "1-8 tests", "H all keys"])

# The H panel: every key and what it does.
ALL_KEYS = [
    ("Enter", "arm (never disarms)"),
    ("Space", "DISARM, same as clicking the red button"),
    ("W / S", "climb / descend. Let go to hover. The first W after arming takes off"),
    ("Q / E", "hover trim down / up 1%. E if it sinks hands off, Q if it climbs"),
    ("Arrows", "lean forward, back, left, right"),
    ("A / D", "turn left / right"),
    ("T", "take control when the video is lost"),
    ("C", "camera: chase, drone camera, ground"),
    ("L", "cut / restore the control link"),
    ("V", "cut / restore the video"),
    ("1 to 8", "start a flight test (1-4 hardware, 5-8 software). 0 is free flight"),
    ("R", "reset, or try the flight test again"),
    ("P", "pause"),
    ("F", "full screen"),
    ("H", "this panel. The simulator waits while it is open"),
    ("Esc", "quit"),
]


# Turn ["mass_g=70", "regulator=ams1117"] into typed values, checked against the defaults.
def parse_settings(pairs):
    defaults, out = default_params(), {}
    for pair in pairs:
        if "=" not in pair:
            sys.exit(f"--set needs name=value, got '{pair}'")
        name, value = pair.split("=", 1)
        if name not in defaults:
            sys.exit(f"Unknown setting '{name}'. Run with --list to see them all.")
        kind = type(defaults[name])
        try:
            if kind is bool:
                out[name] = value.lower() in ("1", "true", "yes", "on")
            elif kind in (int, float):
                out[name] = float(value) if "." in value or kind is float else int(value)
            else:
                out[name] = value
        except ValueError:
            sys.exit(f"{name} needs a number, got '{value}'")
        if name in CHOICES and out[name] not in CHOICES[name]:
            sys.exit(f"{name} must be one of: {', '.join(CHOICES[name])}")
        if name in LIMITS and not LIMITS[name][0] <= out[name] <= LIMITS[name][1]:
            sys.exit(f"{name} must be between {LIMITS[name][0]} and {LIMITS[name][1]}, got {value}")
    return out


def main():
    ap = argparse.ArgumentParser(description="Drone v.0.0 flight simulator")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="change a drone setting")
    ap.add_argument("--list", action="store_true", help="list settings and defaults")
    ap.add_argument("--tests", action="store_true", help="list the flight tests")
    ap.add_argument("--tune", action="store_true", help="print a tuning report for this build (no window)")
    ap.add_argument("--seed", type=int, metavar="N", help="which drone to fly: the same number gives the same "
                    "build errors every time. Left out, each launch is a different drone")
    ap.add_argument("--version", action="version", version=f"Drone v.0.0 Sim: {VERSION}")
    args = ap.parse_args()
    if args.tests:
        for n, t in TESTS.items():
            print(f"  {n}  {t['who']:9s} {t['name']:14s} {t['brief']}")
        return
    if args.list:
        for k, v in default_params().items():
            allowed = " or ".join(CHOICES[k]) if k in CHOICES else "{} to {}".format(*LIMITS[k]) if k in LIMITS else "on or off"
            print(f"  {k:20s} {str(v):10s} ({allowed})")
        return

    settings = parse_settings(args.set)
    if args.tune:
        from drone_sim.hardware.tuning import report
        report(settings)
        return
    import pygame  # imported here so --list and setting errors work without pygame installed
    from drone_sim.view import View

    # Each launch is a different drone unless a seed is given. The flight tests always use their own.
    seed = args.seed if args.seed is not None else random.randrange(1, 10000)
    print(f"Drone {seed}. To fly this exact one again: --seed {seed}")
    sim = Sim(settings, seed)
    cc = CommandCenter(sim)
    tests = FlightTests(cc)
    about = [VERSION, f"Drone {seed} (fly it again with --seed {seed})",
             "Changed settings: " + (", ".join(f"{k}={v}" for k, v in settings.items()) or "none")]

    pygame.init()
    pygame.display.set_caption(f"Drone v.0.0 Sim - by Mori - drone {seed} - {VERSION}")
    windowed = (1100, 700)
    screen = pygame.display.set_mode(windowed, pygame.RESIZABLE)
    view, clock = View(screen), pygame.time.Clock()
    fullscreen = paused = show_keys = False

    names = {pygame.K_w: "w", pygame.K_s: "s", pygame.K_a: "a", pygame.K_d: "d",
             pygame.K_UP: "up", pygame.K_DOWN: "down", pygame.K_LEFT: "left", pygame.K_RIGHT: "right",
             pygame.K_q: "q", pygame.K_e: "e"}

    running = True
    while running:
        frame_dt = min(0.05, clock.tick(60) / 1000)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.MOUSEBUTTONDOWN and view.disarm_box.collidepoint(ev.pos):
                cc.disarm()
            elif ev.type == pygame.KEYDOWN:
                if ev.key in names:
                    cc.pilot.key_down(names[ev.key])
                elif ev.key == pygame.K_ESCAPE:
                    running = False
                elif pygame.K_0 <= ev.key <= pygame.K_8:
                    tests.start(ev.key - pygame.K_0)
                    view.chase_yaw = None
                elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    cc.arm_drone()
                elif ev.key == pygame.K_SPACE:
                    cc.disarm()
                elif ev.key == pygame.K_t:
                    cc.take_control()
                elif ev.key == pygame.K_c:
                    view.cam = {"chase": "fpv", "fpv": "ground", "ground": "chase"}[view.cam]
                elif ev.key == pygame.K_l:
                    sim.link_cut = not sim.link_cut
                    sim.event("Link cut (nothing gets through the bridge)" if sim.link_cut else "Link restored",
                              "bad" if sim.link_cut else "good")
                elif ev.key == pygame.K_v:
                    cc.video.cut = not cc.video.cut
                    sim.event("Video cut (no more frames from the camera)" if cc.video.cut else "Video restored",
                              "bad" if cc.video.cut else "good")
                elif ev.key == pygame.K_f:
                    fullscreen = not fullscreen
                    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN) if fullscreen \
                        else pygame.display.set_mode(windowed, pygame.RESIZABLE)
                    view.screen = screen
                elif ev.key == pygame.K_r:
                    tests.start(tests.n)   # in free flight this is a plain reset
                    view.chase_yaw = None
                elif ev.key == pygame.K_p:
                    paused = not paused
                elif ev.key == pygame.K_h:
                    show_keys = not show_keys
            elif ev.type == pygame.VIDEORESIZE and not fullscreen:
                windowed = ev.size
                screen = pygame.display.set_mode(windowed, pygame.RESIZABLE)
                view.screen = screen

        if not paused and not show_keys:
            pressed = pygame.key.get_pressed()
            cc.update({name for key, name in names.items() if pressed[key]}, frame_dt)
            cc.advance(frame_dt)
            sim.collide(BOXES)
            tests.update(frame_dt)

        view.render(sim, frame_dt, cc.video, tests.rings())
        view.hud(sim, paused, KEY_HELP, cc, tests)
        if show_keys:
            view.key_panel(ALL_KEYS, about)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()

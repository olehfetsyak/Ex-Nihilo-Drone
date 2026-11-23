# Live window test: opens the real simulator window and flies it with real key presses, like a
# person at the keyboard. Windows only (it uses the Windows keyboard and mouse calls).
#
#     python tools/live_window_test.py              everything: free flight, then all eight flight tests
#     python tools/live_window_test.py free         only the free-flight checks
#     python tools/live_window_test.py 1,7,6b       only those flight tests (6b = test 6, taking control with T)
#
# Don't touch the keyboard or mouse while it runs: keys are only sent while the simulator window
# is in front, and the run stops if it loses the focus. Takes about five minutes. Screenshots go
# to a folder in the temp directory, printed at the start.
#
# The scripted pilot reads the simulator's true position, the way a person's eyes would.
import ctypes, math, os, sys, tempfile, threading, time, traceback

if sys.platform != "win32":
    sys.exit("This test only runs on Windows.")
from ctypes import wintypes

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
only = sys.argv[1].split(",") if len(sys.argv) > 1 else None
out = os.path.join(tempfile.gettempdir(), "drone_sim_live_test")
os.makedirs(out, exist_ok=True)
print("screenshots:", out)
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import pygame
import run_sim
from drone_sim import view as view_mod
from drone_sim.quat import euler

u = ctypes.windll.user32
S = {"frames": 0}
shots = []

Orig = run_sim.FlightTests
class Spy(Orig):
    def __init__(self, cc):
        super().__init__(cc)
        S["ft"], S["cc"], S["sim"] = self, cc, cc.sim
run_sim.FlightTests = Spy

_vinit = view_mod.View.__init__
def vinit(self, screen):
    _vinit(self, screen)
    S["view"] = self
view_mod.View.__init__ = vinit

_flip = pygame.display.flip
def flip():
    _flip()
    S["frames"] += 1
    if shots:
        pygame.image.save(pygame.display.get_surface(), os.path.join(out, shots.pop(0)))
pygame.display.flip = flip

VK = {c: ord(c.upper()) for c in "wsadqertplvcfh012345678"}
VK.update(space=0x20, enter=0x0D, esc=0x1B, left=0x25, up=0x26, right=0x27, down=0x28)
EXT = {"left", "up", "right", "down"}
down_now = set()
results = []


class Lost(Exception):
    pass


def log(*a):
    print(*a, flush=True)


def check(name, ok, detail=""):
    results.append((name, bool(ok)))
    log(("  ok   " if ok else "  FAIL ") + name + (f"  [{detail}]" if detail else ""))


def hwnd():
    return pygame.display.get_wm_info()["window"]


def focused():
    return u.GetForegroundWindow() == hwnd()


def key(name, down):
    if down and not focused():
        raise Lost("sim window lost the focus")
    vk = VK[name]
    u.keybd_event(vk, u.MapVirtualKeyW(vk, 0), (0 if down else 2) | (1 if name in EXT else 0), 0)
    (down_now.add if down else down_now.discard)(name)


def hold(keys):
    keys = set(keys)
    for k in list(down_now - keys):
        key(k, False)
    for k in keys - down_now:
        key(k, True)


def tap(name, wait=0.25):
    hold([])
    key(name, True); time.sleep(0.07); key(name, False); time.sleep(wait)


def shot(name):
    shots.append(name)
    while name in shots:
        time.sleep(0.02)


def fly(fn, secs, until=None):
    t0 = time.time()
    while time.time() - t0 < secs:
        hold(fn() if callable(fn) else fn)
        time.sleep(1 / 30)
        if until and until():
            break
    return time.time() - t0


def pilot(tx, ty, tz):
    sim, held = S["sim"], set()
    z, vz = sim.pos[2], sim.vel[2]
    if z < tz - 0.2 and vz < 0.6: held.add("w")
    elif z > tz + 0.2 and vz > -0.6: held.add("s")
    wx = (tx - sim.pos[0]) - 1.2 * sim.vel[0]
    wy = (ty - sim.pos[1]) - 1.2 * sim.vel[1]
    if wx > 0.3: held.add("up")
    elif wx < -0.3: held.add("down")
    if wy > 0.3: held.add("left")
    elif wy < -0.3: held.add("right")
    return held


# taps whose length depends on how far off it is, the way a person feathers on/off keys
def human(tx, ty, tz):
    sim, held = S["sim"], set()
    phase = (time.time() % 0.3) / 0.3
    def axis(u, pos, neg):
        if abs(u) > 0.12 and phase < min(1.0, abs(u)):
            held.add(pos if u > 0 else neg)
    axis(1.2 * (tz - sim.pos[2]) - 0.9 * sim.vel[2], "w", "s")
    axis(0.5 * (tx - sim.pos[0]) - 0.7 * sim.vel[0], "up", "down")
    axis(0.5 * (ty - sim.pos[1]) - 0.7 * sim.vel[1], "left", "right")
    return held


def land(disarm="space"):
    sim = S["sim"]
    fly(lambda: pilot(sim.pos[0], sim.pos[1], 0.5), 15, lambda: sim.pos[2] < 0.7)
    fly(["s"], 10, lambda: sim.on_ground)
    hold([])
    if disarm:
        tap(disarm, 0.4)


def arm_and_take_off(z=1.5):
    sim = S["sim"]
    time.sleep(0.8)
    tap("enter", 0.5)
    fly(lambda: pilot(0, 0, z), 4)
    return sim.armed


def free_flight():
    sim, cc, view = S["sim"], S["cc"], S["view"]
    roll = lambda: euler(sim.q)[0] * 57.3
    log("free flight")
    shot("01_start.png")
    time.sleep(0.5)
    tap("enter", 0.5)
    check("Enter arms", sim.armed)
    tap("w", 0.1)
    fly(lambda: pilot(0, 0, 1.5), 5)
    check("W takes off, W/S hold a height", 1.0 < sim.pos[2] < 2.0, f"z={sim.pos[2]:.2f}")
    tap("e", 0.1); tap("e", 0.1)
    check("E trims up", abs(cc.pilot.trim - 0.02) < 1e-6, cc.pilot.trim)
    tap("q", 0.1); tap("q", 0.1)
    check("Q trims down", abs(cc.pilot.trim) < 1e-6, cc.pilot.trim)
    for k, axis, sgn in (("up", 0, 1), ("down", 0, -1), ("left", 1, 1), ("right", 1, -1)):
        fly(lambda: pilot(0, 0, 1.5), 3)
        fly([], 1.0)   # let go and level out, so the push starts from a settled hover
        v0 = sim.vel[axis]
        fly([k], 0.8)
        check(f"arrow {k} moves the right way", (sim.vel[axis] - v0) * sgn > 0.8, f"dv={sim.vel[axis] - v0:.1f}")
    fly(lambda: pilot(0, 0, 1.5), 4)
    cams = []
    for _ in range(3):
        tap("c", 0.3); cams.append(view.cam); shot(f"02_cam_{view.cam}.png")
    check("C cycles the cameras", cams == ["fpv", "ground", "chase"], cams)
    fly(lambda: pilot(0, 0, 1.5), 1)
    tap("p", 0.3); t0 = sim.t; time.sleep(0.5)
    check("P pauses", sim.t == t0)
    shot("03_paused.png")
    tap("p", 0.3)
    check("P resumes", sim.t > t0)
    tap("h", 0.3); t0 = sim.t; time.sleep(0.5)
    check("H opens the key panel and the sim waits", sim.t == t0)
    shot("03b_keys.png")
    tap("h", 0.3)
    check("H closes it and the sim carries on", sim.t > t0)

    log("video loss: hover, ask, take control")
    fly(lambda: pilot(0, 0, 1.5), 2)
    tap("v", 0.9)
    check("V cuts video, safety takes over", cc.safety.source == "safety" and not cc.safety.video_ok)
    fly(["right"], 0.8)
    check("keys locked while it asks", abs(roll()) < 6 and cc.safety.source == "safety", f"roll={roll():.0f}")
    shot("04_video_prompt.png")
    hold([])
    tap("t", 0.2)
    check("T takes control", cc.safety.takeover)
    fly(["left"], 0.6)
    check("keys fly without video after T", roll() < -10, f"roll={roll():.0f}")
    shot("05_takeover.png")
    fly(lambda: pilot(0, 0, 1.5), 3)
    tap("v", 0.5)
    check("video back clears the takeover", cc.safety.video_ok and not cc.safety.takeover)

    log("video loss: nobody answers, so it lands")
    fly(lambda: pilot(0, 0, 1.5), 3)
    tap("v", 0.3)
    hold([])
    time.sleep(3.0)
    check("still hovering after 3 s", cc.safety.source == "safety" and not cc.safety.landing, f"z={sim.pos[2]:.2f}")
    time.sleep(3.0)
    check("landing after 5 s", cc.safety.landing)
    shot("06_video_landing.png")
    t0 = time.time()
    while not sim.on_ground and time.time() - t0 < 20:
        time.sleep(0.05)
    check("it lands itself without crashing", sim.on_ground and not sim.crashed, f"after {time.time() - t0:.1f} s")
    tap("space", 0.4)
    check("Space disarms with video down", not sim.armed)
    tap("enter", 0.4)
    check("can't arm without video", not cc.arm)
    tap("v", 0.5)

    log("link loss")
    tap("r", 0.3)
    check("R resets", sim.t < 1.0, f"t={sim.t:.2f}")
    arm_and_take_off()
    tap("l", 1.0)
    check("L cuts the link, drone goes to failsafe", sim.failsafe and not cc.safety.link_ok)
    shot("07_link_lost.png")
    tap("l", 0.6)
    check("L restores it, control comes back", not sim.failsafe and cc.safety.link_ok)
    fly(lambda: pilot(sim.pos[0], sim.pos[1], 1.2), 3)
    y0 = euler(sim.q)[2]
    fly(["d"], 0.6)
    check("D turns right", euler(sim.q)[2] < y0 - 0.3)
    y1 = euler(sim.q)[2]
    fly(["a"], 1.2)
    check("A turns left", euler(sim.q)[2] > y1 + 0.3)
    fly(["s"], 10, lambda: sim.on_ground)
    hold([])
    check("S brings it down, motors still running", sim.on_ground and sim.armed and not sim.crashed)
    box = view.disarm_box
    pt = wintypes.POINT(box.centerx, box.centery)
    u.ClientToScreen(hwnd(), ctypes.byref(pt))
    u.SetCursorPos(pt.x, pt.y); time.sleep(0.2)
    if focused():
        u.mouse_event(2, 0, 0, 0, 0); time.sleep(0.05); u.mouse_event(4, 0, 0, 0, 0)
    time.sleep(0.5)
    check("clicking DISARM stops the motors", not sim.armed and not cc.arm)
    size0 = pygame.display.get_surface().get_size()
    tap("f", 1.2)
    size1 = pygame.display.get_surface().get_size()
    shot("08_fullscreen.png")
    tap("f", 1.2)
    check("F goes full screen and back", size1 != size0 and pygame.display.get_surface().get_size() == size0, f"{size0} {size1}")
    u.SetForegroundWindow(hwnd()); time.sleep(0.5)


def flight_test(n, variant=""):
    sim, cc, ft = S["sim"], S["cc"], S["ft"]
    tap(str(n), 0.4)
    if ft.n != n:
        check(f"key {n} starts test {n}", False); return
    name = ft.test["name"]
    shot(f"t{n}{variant}_start.png")
    time.sleep(0.5)
    tap("enter", 0.5)
    tap("w", 0.05)
    done = lambda: ft.result is not None
    kind = ft.test["check"].__name__
    if kind == "hover":
        fly(lambda: human(0, 0, 1.25), 60, done)
        log(f"    trim ended at {cc.pilot.trim:+.2f}, took {sim.t:.0f} s")
    elif kind == "stay_close":
        fly(lambda: pilot(0, 0, 1.5), 45, done)
    elif kind == "weak_battery":
        fly(["w"], 15, lambda: ft.stage == 1 or done())
        fly(["s"], 15, lambda: sim.on_ground or done()); hold([]); tap("space", 0.5)
    elif kind == "link_drop":
        fly(lambda: pilot(0, 0, 4.6), 25, lambda: ft.stage == 1 or done())
        log(f"    link cut at z={sim.pos[2]:.1f} vz={sim.vel[2]:.1f}")
        fly([], 10, lambda: ft.stage == 2 or done())
        log(f"    link back at z={sim.pos[2]:.1f} vz={sim.vel[2]:.1f}")
        fly(["w"], 3, lambda: sim.vel[2] > -0.3)
        fly(["s"], 15, lambda: sim.on_ground or done()); hold([]); tap("space", 0.5)
    elif kind == "video_drop":
        fly(lambda: pilot(14, 0, 1.5), 25, lambda: ft.stage == 1 or done())
        x0 = sim.pos[0]
        if variant == "b":
            time.sleep(1.0); hold([]); tap("t", 0.1)
            fly(["down"], 1.2)
            fly(["s"], 15, lambda: sim.on_ground or done())
        else:
            fly([], 40, lambda: sim.on_ground or done())
        log(f"    travelled {sim.pos[0] - x0:.1f} m after the video cut")
        shot(f"t{n}{variant}_down.png")
        hold([]); tap("space", 0.5)
    else:
        x, y, _, r = ft.test["rings"][0]
        def go():
            d = math.hypot(sim.pos[0] - x, sim.pos[1] - y)
            near = d < 0.6 and math.hypot(sim.vel[0], sim.vel[1]) < 0.5
            return {"s"} if near or (ft.flown and sim.pos[2] < 0.5 and d < 1.2) else human(x, y, 1.2)
        took = fly(go, 70, lambda: (sim.on_ground and ft.flown) or done())
        log(f"    {took:.0f} s to get down, {math.hypot(sim.pos[0] - x, sim.pos[1] - y):.1f} m from the middle")
        if not sim.on_ground:
            fly(["s"], 10, lambda: sim.on_ground)
        hold([]); tap("space", 0.5)
    hold([])
    time.sleep(0.4)
    shot(f"t{n}{variant}_result.png")
    check(f"test {n}{variant} {name}: {ft.result}", ft.result and ft.result[0])


def script():
    try:
        while S["frames"] < 30 or "view" not in S:
            time.sleep(0.05)
        h = hwnd()
        u.ShowWindow(h, 5)
        for _ in range(5):
            if focused():
                break
            u.keybd_event(0x12, 0, 0, 0); u.SetForegroundWindow(h); u.keybd_event(0x12, 0, 2, 0)
            time.sleep(0.4)
        if not focused():
            raise Lost("could not bring the sim window to the front")
        log("window has the focus")
        if not only or "free" in only:
            free_flight()
        for n in range(1, 9):
            if not only or str(n) in only:
                flight_test(n)
        if not only or "6b" in only:
            flight_test(6, "b")
        if not only:
            ft = S["ft"]
            tap("r", 0.5)
            check("R retries the same test", ft.n == 6 and ft.result is None and S["sim"].t < 1.0)
            tap("0", 0.4)
            check("0 goes back to free flight", ft.test is None)
        shot("99_end.png")
    except Lost as e:
        log("ABORTED:", e)
    except Exception:
        log(traceback.format_exc())
    finally:
        for k in list(down_now):
            u.keybd_event(VK[k], 0, 2 | (1 if k in EXT else 0), 0)
        bad = [n for n, ok in results if not ok]
        log(f"{len(results) - len(bad)}/{len(results)} checks ok")
        for n in bad:
            log("  failed:", n)
        S["failed"] = len(bad) or not results
        pygame.event.post(pygame.event.Event(pygame.QUIT))


threading.Thread(target=script, daemon=True).start()
sys.argv = ["run_sim.py"]
run_sim.main()
sys.exit(1 if S.get("failed", True) else 0)

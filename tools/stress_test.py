# Stress test: runs the real program loop with no visible window and throws random keys, clicks,
# window resizes and flight-test starts at it. Reports any error, any broken number in the drone's
# state, and how long frames take.
#
#     python tools/stress_test.py                          seed 1, 5 simulated minutes
#     python tools/stress_test.py 7 10                     seed 7, 10 simulated minutes
#     python tools/stress_test.py 7 10 --set video_loss=60 --set wind=4     with settings
#
# A random P or H can pause it for a long stretch, so "in the air" varies a lot between seeds.
import math, os, random, sys, time, traceback

os.environ["SDL_VIDEODRIVER"] = "dummy"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the repo folder
seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1
minutes = float(sys.argv[2]) if len(sys.argv) > 2 else 5
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import pygame
import run_sim
from drone_sim import command_center, flight_tests

rng = random.Random(seed)
FLY = [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT]
TAPS = [pygame.K_q, pygame.K_e, pygame.K_RETURN, pygame.K_SPACE, pygame.K_t, pygame.K_c, pygame.K_l, pygame.K_v,
        pygame.K_f, pygame.K_r, pygame.K_p, pygame.K_h, pygame.K_9, pygame.K_z, pygame.K_F1, pygame.K_TAB] + \
       [pygame.K_0 + n for n in range(9)]
held = set()
slow = []


class Pressed:
    def __getitem__(self, k):
        return k in held


class Clock:
    def tick(self, fps=0):
        return rng.choice([16, 16, 17, 16, 33, 8, 120])   # mostly 60 fps, sometimes a hitch


seen = {}
orig_cc = command_center.CommandCenter.__init__
def cc_init(self, sim):
    orig_cc(self, sim); seen["cc"] = self
command_center.CommandCenter.__init__ = cc_init
orig_ft = flight_tests.FlightTests.__init__
def ft_init(self, cc):
    orig_ft(self, cc); seen["ft"] = self
flight_tests.FlightTests.__init__ = ft_init

frames, t_last, frame_ms, results, armed_frames, air_frames, worst = 0, None, [], {}, 0, 0, 0.0
target = int(minutes * 60 * 60)


def flip():
    global frames, t_last, worst
    now = time.perf_counter()
    if t_last is not None:
        frame_ms.append((now - t_last) * 1000)
        if (now - t_last) > 0.08 and len(slow) < 25:
            s_ = seen['cc'].sim; slow.append((round((now - t_last) * 1000), pygame.display.get_surface().get_size(), [round(v) for v in s_.pos], s_.crashed, seen['ft'].n, round(s_.t, 1), round(s_.loop_load, 2)))
    frames += 1
    sim, ft = seen["cc"].sim, seen["ft"]
    vals = sim.pos + sim.vel + sim.q + sim.w + [sim.vload, sim.soc] + sim.mot
    if not all(math.isfinite(v) for v in vals):
        raise RuntimeError(f"non-finite state at frame {frames}: {vals}")
    armed_frames_inc = sim.armed
    globals()["armed_frames"] += armed_frames_inc
    globals()["air_frames"] += sim.pos[2] > 0.3
    worst = max(worst, abs(sim.pos[0]), abs(sim.pos[1]), sim.pos[2])
    if ft.result and ft.n:
        results[(ft.n, ft.result[0])] = results.get((ft.n, ft.result[0]), 0) + 1
    if frames >= target:
        pygame.event.post(pygame.event.Event(pygame.QUIT))
    # random input. Flight keys change a few times a second; taps are rarer so flights get going
    if rng.random() < 0.06:
        held.clear()
        for k in rng.sample(FLY, rng.choice([0, 0, 1, 1, 2, 3])):
            held.add(k); pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=k))
        if rng.random() < 0.5:
            held.add(pygame.K_w); pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_w))
    r = rng.random()
    if r < 0.004:
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=rng.choice(TAPS)))
    elif r < 0.012:
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RETURN))   # arm often, or nothing ever flies
    elif r < 0.0125:
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=(rng.randrange(0, 1200), rng.randrange(0, 800)), button=1))
    elif r < 0.013:
        pygame.event.post(pygame.event.Event(pygame.VIDEORESIZE, size=rng.choice([(1, 1), (120, 80), (640, 360), (1100, 700), (2560, 1440)]), w=0, h=0))
    t_last = time.perf_counter()


pygame.display.flip = flip
pygame.key.get_pressed = lambda: Pressed()
pygame.time.Clock = Clock
sys.argv = ["run_sim.py"] + sys.argv[3:]
try:
    run_sim.main()
    status = "ran to the end"
except BaseException:
    status = "CRASHED:\n" + traceback.format_exc()
frame_ms.sort()
print(f"seed {seed}: {status}. {frames} frames ({frames / 3600:.1f} sim min), armed {armed_frames / max(1, frames) * 100:.0f}% of the time, "
      f"in the air {air_frames / max(1, frames) * 100:.0f}%, furthest {worst:.0f} m. "
      f"frame time median {frame_ms[len(frame_ms) // 2]:.1f} ms, 99th percentile {frame_ms[int(len(frame_ms) * 0.99)]:.1f} ms, worst {frame_ms[-1]:.0f} ms. "
      f"test results seen: {sorted(set(results))}")

for row in slow: print('slow frame', row)
sys.exit(0 if status == "ran to the end" else 1)

# 3D view and HUD drawn with pygame: plain perspective projection, painted far to near.
import math

import pygame

from .quat import euler, qrot
from .hardware.drone import MOTORS, R2D
from .world import BOXES

NEAR = 0.03
INK = (19, 36, 46)
ACCENT = (214, 18, 126)
LAND = (238, 226, 205)
SKY_TOP, SKY_BOTTOM = (159, 211, 228), (226, 238, 236)
GRID_MAJOR, GRID_MINOR = (214, 180, 150), (230, 214, 192)
GOOD, BAD, WARN = (34, 139, 84), (196, 40, 40), (200, 120, 0)


def _norm(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) or 1.0
    return [v[0] / n, v[1] / n, v[2] / n]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


class Camera:
    def __init__(self, pos, fwd, up, fov_deg, w, h):
        self.pos, self.w, self.h = pos, w, h
        self.f = _norm(fwd)
        self.r = _norm(_cross(self.f, up))
        self.u = _cross(self.r, self.f)
        self.foc = h / 2 / math.tan(fov_deg * math.pi / 360)

    def view(self, p):
        d = [p[0] - self.pos[0], p[1] - self.pos[1], p[2] - self.pos[2]]
        return [sum(d[i] * self.r[i] for i in range(3)), sum(d[i] * self.u[i] for i in range(3)),
                sum(d[i] * self.f[i] for i in range(3))]

    def proj(self, v):
        return (self.w / 2 + v[0] / v[2] * self.foc, self.h / 2 - v[1] / v[2] * self.foc)

    def dist(self, p):
        return math.dist(p, self.pos)


class View:
    def __init__(self, screen):
        self.screen = screen
        self.font = pygame.font.SysFont("consolas,menlo,dejavusansmono,monospace", 15)
        self.big = pygame.font.SysFont("segoeui,helvetica,dejavusans,sans", 22, bold=True)
        self.cam = "chase"
        self.chase_yaw = None
        self._sky = None
        self.disarm_box = pygame.Rect(0, 0, 0, 0)

    # ---------------------------------------------------------------- drawing helpers
    def _clip(self, c, pts):
        vs = [c.view(p) for p in pts]
        out = []
        for i, a in enumerate(vs):
            b = vs[(i + 1) % len(vs)]
            if a[2] >= NEAR:
                out.append(a)
            if (a[2] >= NEAR) != (b[2] >= NEAR):
                t = (NEAR - a[2]) / (b[2] - a[2])
                out.append([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, NEAR])
        return [c.proj(v) for v in out]

    # Cut a projected polygon down to the screen (plus a small border). Without this a big shape
    # close to the camera gets corners millions of pixels away, and filling it takes many times longer.
    def _trim(self, pts, pad=4):
        w, h = self.screen.get_size()
        for axis, limit, keep_below in ((0, -pad, False), (0, w + pad, True), (1, -pad, False), (1, h + pad, True)):
            out = []
            for i, a in enumerate(pts):
                b = pts[(i + 1) % len(pts)]
                a_in, b_in = (a[axis] <= limit) == keep_below, (b[axis] <= limit) == keep_below
                if a_in:
                    out.append(a)
                if a_in != b_in:
                    t = (limit - a[axis]) / (b[axis] - a[axis])
                    out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            pts = out
            if len(pts) < 3:
                return []
        return pts

    def poly(self, c, pts, fill=None, stroke=None, width=1):
        s = self._trim(self._clip(c, pts))
        if len(s) < 3:
            return
        if fill:
            pygame.draw.polygon(self.screen, fill, s)
        if stroke:
            pygame.draw.polygon(self.screen, stroke, s, width)

    def line(self, c, a, b, color, width=1):
        va, vb = c.view(a), c.view(b)
        if va[2] < NEAR and vb[2] < NEAR:
            return
        if va[2] < NEAR or vb[2] < NEAR:
            t = (NEAR - va[2]) / (vb[2] - va[2])
            mid = [va[0] + (vb[0] - va[0]) * t, va[1] + (vb[1] - va[1]) * t, NEAR]
            if va[2] < NEAR:
                va = mid
            else:
                vb = mid
        pygame.draw.line(self.screen, color, c.proj(va), c.proj(vb), max(1, round(width)))

    def _sky_surface(self, w, h):
        if self._sky is None or self._sky.get_size() != (w, h):
            self._sky = pygame.Surface((w, h))
            for y in range(h):
                t = y / max(1, h - 1)
                col = [round(SKY_TOP[i] + (SKY_BOTTOM[i] - SKY_TOP[i]) * t) for i in range(3)]
                pygame.draw.line(self._sky, col, (0, y), (w, y))
        return self._sky

    # ---------------------------------------------------------------- world
    def draw_ring(self, c, ring):
        x, y, z, r = ring
        pts = [[x + math.cos(k / 32 * 2 * math.pi) * r, y + math.sin(k / 32 * 2 * math.pi) * r, z + 0.006] for k in range(33)]
        for a, b in zip(pts, pts[1:]):
            self.line(c, a, b, ACCENT, 3)

    def draw_world(self, c, sim, show_drone, rings=()):
        self.screen.blit(self._sky_surface(c.w, c.h), (0, 0))
        gx, gy = round(c.pos[0]), round(c.pos[1])
        self.poly(c, [[gx - 300, gy - 300, 0], [gx + 300, gy - 300, 0], [gx + 300, gy + 300, 0], [gx - 300, gy + 300, 0]], LAND)
        for i in range(-25, 26):
            X, Y = gx + i, gy + i
            self.line(c, [X, gy - 25, 0], [X, gy + 25, 0], GRID_MAJOR if X % 5 == 0 else GRID_MINOR)
            self.line(c, [gx - 25, Y, 0], [gx + 25, Y, 0], GRID_MAJOR if Y % 5 == 0 else GRID_MINOR)
        pad = [[math.cos(k / 24 * 2 * math.pi) * 0.6, math.sin(k / 24 * 2 * math.pi) * 0.6, 0.002] for k in range(24)]
        self.poly(c, pad, (222, 214, 198), INK, 2)
        for a, b in [([-0.3, 0, 0.003], [0.3, 0, 0.003]), ([0.3, 0, 0.003], [0.15, 0.12, 0.003]),
                     ([0.3, 0, 0.003], [0.15, -0.12, 0.003])]:
            self.line(c, a, b, INK, 2)
        if show_drone:
            p, hz = sim.pos, max(0.05, sim.pos[2])
            rad = 0.07 * (1 + hz * 0.15)
            shade = max(150, min(215, round(160 + hz * 10)))
            self.poly(c, [[p[0] + math.cos(k / 16 * 2 * math.pi) * rad, p[1] + math.sin(k / 16 * 2 * math.pi) * rad, 0.004]
                          for k in range(16)], (shade, shade - 8, shade - 20))
        objs = [(c.dist([b["x"], b["y"], b["sz"] / 2]), lambda b=b: self.draw_box(c, b)) for b in BOXES]
        objs += [(c.dist(list(ring[:3])), lambda ring=ring: self.draw_ring(c, ring)) for ring in rings]
        if show_drone:
            objs.append((c.dist(sim.pos), lambda: self.draw_drone(c, sim)))
        for _, draw in sorted(objs, key=lambda o: -o[0]):
            draw()

    def draw_box(self, c, b):
        x0, x1, y0, y1, h = b["x"] - b["sx"] / 2, b["x"] + b["sx"] / 2, b["y"] - b["sy"] / 2, b["y"] + b["sy"] / 2, b["sz"]
        faces = [([[x0, y0, 0], [x1, y0, 0], [x1, y0, h], [x0, y0, h]], [0, -1, 0], 0.82),
                 ([[x1, y0, 0], [x1, y1, 0], [x1, y1, h], [x1, y0, h]], [1, 0, 0], 0.7),
                 ([[x1, y1, 0], [x0, y1, 0], [x0, y1, h], [x1, y1, h]], [0, 1, 0], 0.62),
                 ([[x0, y1, 0], [x0, y0, 0], [x0, y0, h], [x0, y1, h]], [-1, 0, 0], 0.75),
                 ([[x0, y0, h], [x1, y0, h], [x1, y1, h], [x0, y1, h]], [0, 0, 1], 1.0)]
        for pts, n, shade in faces:
            centre = [sum(q[i] for q in pts) / 4 for i in range(3)]
            to_cam = [c.pos[i] - centre[i] for i in range(3)]
            if sum(to_cam[i] * n[i] for i in range(3)) <= 0:
                continue  # facing away
            v = round(200 * shade)
            self.poly(c, pts, (v, round(v * 0.86), round(v * 0.7)), (120, 110, 100), 1)

    def draw_drone(self, c, sim):
        L = sim.p["arm_mm"] / 1000 * math.sqrt(0.5)

        def body(v):
            w = qrot(sim.q, v)
            return [sim.pos[0] + w[0], sim.pos[1] + w[1], sim.pos[2] + w[2]]

        motors = [[sx * L, sy * L, 0] for sx, sy, _ in MOTORS]
        for i, m in enumerate(motors):  # front arms pink, rear arms dark
            self.line(c, body([0, 0, 0.004]), body([m[0], m[1], 0.004]), ACCENT if i < 2 else INK, 4)
        self.poly(c, [body(v) for v in ([0.022, 0.014, 0.012], [0.022, -0.014, 0.012], [-0.022, -0.014, 0.012], [-0.022, 0.014, 0.012])], INK)
        self.poly(c, [body(v) for v in ([0.03, 0.009, 0.016], [0.03, -0.009, 0.016], [0.018, -0.009, 0.026], [0.018, 0.009, 0.026])], (242, 194, 48))
        for i, m in enumerate(motors):
            spin = sim.mot[i]
            fill = tuple(round(255 - (255 - ch) * (0.12 + 0.35 * spin)) for ch in INK)
            self.poly(c, [body([m[0] + math.cos(k / 14 * 2 * math.pi) * 0.02, m[1] + math.sin(k / 14 * 2 * math.pi) * 0.02, 0.012])
                          for k in range(14)], fill, (90, 100, 106), 1)

    # ---------------------------------------------------------------- cameras
    def render(self, sim, frame_dt, video, rings=()):
        w, h = self.screen.get_size()
        p = sim.p
        if self.cam == "fpv":   # the newest frame the laptop has: delayed, and frozen if the video is cut
            _, pos, q = video.frame or (0, sim.pos, sim.q)
            t = p["cam_tilt"] * math.pi / 180
            f, up = qrot(q, [math.cos(t), 0, math.sin(t)]), qrot(q, [-math.sin(t), 0, math.cos(t)])
            cp = qrot(q, [0.03, 0, 0.022])
            c = Camera([pos[0] + cp[0], pos[1] + cp[1], pos[2] + cp[2] + 0.001], f, up, p["cam_fov"], w, h)
            self.draw_world(c, sim, False, rings)
            pygame.draw.line(self.screen, (255, 255, 255), (w / 2 - 12, h / 2), (w / 2 + 12, h / 2), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (w / 2, h / 2 - 12), (w / 2, h / 2 + 12), 2)
            if video.cut:
                label = self.big.render("NO VIDEO", True, BAD)
                self.screen.blit(label, label.get_rect(center=(w / 2, h * 0.25)))
        elif self.cam == "ground":
            eye = [-2.5, 3.5, 1.4]
            to = [sim.pos[i] - eye[i] for i in range(3)]
            fov = max(12, min(70, 140 / max(1, math.dist(sim.pos, eye))))
            c = Camera(eye, to, [0, 0, 1], fov, w, h)
            self.draw_world(c, sim, True, rings)
        else:  # chase: fixed distance behind, turning smoothly with the drone
            yaw = euler(sim.q)[2]
            if self.chase_yaw is None:
                self.chase_yaw = yaw
            dy = (yaw - self.chase_yaw + math.pi) % (2 * math.pi) - math.pi
            self.chase_yaw += dy * (1 - math.exp(-frame_dt * 3))
            eye = [sim.pos[0] - math.cos(self.chase_yaw) * 0.75, sim.pos[1] - math.sin(self.chase_yaw) * 0.75,
                   max(0.12, sim.pos[2] + 0.3)]
            c = Camera(eye, [sim.pos[0] - eye[0], sim.pos[1] - eye[1], sim.pos[2] + 0.05 - eye[2]], [0, 0, 1], 62, w, h)
            self.draw_world(c, sim, True, rings)

    # ---------------------------------------------------------------- HUD
    def _chip(self, text, x, y, color=INK, align="left"):
        surf = self.font.render(text, True, color)
        r = surf.get_rect()
        box = pygame.Rect(0, 0, r.w + 12, r.h + 6)
        if align == "right":
            box.topright = (x, y)
        else:
            box.topleft = (x, y)
        pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=5)
        self.screen.blit(surf, (box.x + 6, box.y + 3))
        return box

    def hud(self, sim, paused, key_help, cc, tests=None):
        w, h = self.screen.get_size()
        p, safety = sim.p, cc.safety
        # top left: state and link health, as the Command Center sees it
        x = 10
        age = sim.t - sim.telem_t
        link = ("Link: lost", BAD) if not safety.link_ok else ("Link: weak", WARN) if age > 0.1 else ("Link: ok", INK)
        video = ("Video: lost", BAD) if not safety.video_ok else (f"Video: {round(p['video_delay_ms'])} ms late", INK)
        items = [("ARMED", GOOD) if sim.telem["armed"] else ("disarmed", INK), link, video]
        if safety.source == "safety":
            what = "drone failsafe" if not safety.link_ok else "landing" if safety.landing else "hover"
            items.append((f"SAFETY: {safety.reason} -> {what}, keys locked", BAD))
        elif safety.takeover:
            items.append(("no video: pilot has control", WARN))
        elif not (safety.link_ok and safety.video_ok):
            items.append(("keys locked", BAD))
        if safety.low_battery:
            items.append(("LOW BATTERY: land now", BAD))
        # the laptop can't see these while the link is down, so they are simulator-only
        if sim.failsafe:
            items.append(("sim only: drone in failsafe", BAD))
        if not sim.fc_on:
            items.append(("sim only: FC rebooting", BAD))
        if paused:
            items.append(("PAUSED", WARN))
        for text, col in items:
            x = self._chip(text, x, 10, col).right + 6
        # top right: numbers
        # first what the real Command Center will have (flight data from the drone, and what it sends),
        # then what only the simulator knows
        tel, dim = sim.telem, (110, 118, 122)
        rows = [(f"battery {tel['vbat']:.2f} V", BAD if tel["vbat"] < 3.5 else INK),
                (f"roll {round(tel['roll'] * R2D)}  pitch {round(tel['pitch'] * R2D)}", INK),
                (f"throttle {round(sim.rc['thr'] * 100)}%", INK),
                (f"hover trim {round(cc.pilot.trim * 100):+d}%", INK),
                (f"sim only: alt {sim.pos[2]:.2f} m", dim), (f"sim only: {math.hypot(*sim.vel):.1f} m/s", dim),
                (f"sim only: CPU {round(sim.loop_load * 100)}%", BAD if sim.loop_load > 1 else dim)]
        y = 10
        for text, col in rows:
            y = self._chip(text, w - 10, y, col, "right").bottom + 4
        # bottom left: recent events
        recent = [e for e in sim.events if sim.t - e["t"] < 5][-3:]
        y = h - 40 - 26 * len(recent)
        for e in recent:
            col = {"good": GOOD, "bad": BAD}.get(e["kind"], INK)
            y = self._chip(e["msg"], 10, y, col).bottom + 4
        # stick boxes: what the keyboard is sending
        self._stick(w - 190, h - 130, sim.rc["yaw"], sim.rc["thr"] * 2 - 1, "thr/yaw")
        self._stick(w - 95, h - 130, sim.rc["roll"], sim.rc["pitch"], "pitch/roll")
        self._pad_pointer(w - 285, h - 130, sim)
        # flight test: what to do, how it's going, and the result
        if tests and tests.test:
            t = tests.test
            lines = [(self.big, f"Test {tests.n} ({t['who']}): {t['name']}", INK), (self.font, t["brief"], INK)]
            if tests.result:
                lines.append((self.big, ("PASS: " if tests.result[0] else "FAIL: ") + tests.result[1],
                              GOOD if tests.result[0] else BAD))
                lines.append((self.font, "R to try again, 0 for free flight, 1-8 for another test", INK))
            elif tests.note:
                lines.append((self.font, tests.note, ACCENT))
            surfs = [f.render(text, True, col) for f, text, col in lines]
            box = pygame.Rect(0, 0, max(s.get_width() for s in surfs) + 28, sum(s.get_height() + 3 for s in surfs) + 14)
            box.midtop = (w / 2, 44)
            pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=10)
            ty = box.y + 8
            for s in surfs:
                self.screen.blit(s, s.get_rect(midtop=(w / 2, ty)))
                ty += s.get_height() + 3
        # centre banner
        msg = None
        if sim.crashed:
            msg = (sim.crash_msg, "Press R to reset")
        elif not sim.fc_on:
            msg = ("Flight controller browned out", f"Regulator output dropped below {p['brownout_v']} V. Rebooting...")
        elif not sim.armed and sim.rc["arm"] and sim.arm_block:
            msg = ("Arm switch is on but arming is blocked", "Press Space, then Enter, to flip it off and on")
        elif not sim.armed and sim.t > 0.6 and sim.pos[2] < 0.01 and not sim.rc["arm"]:
            msg = ("Press Enter to arm", "Hold W to take off. After landing, press Space to stop the motors")
        if safety.source == "safety" and safety.link_ok:   # video lost: ask the pilot
            if safety.landing:
                msg = ("Video lost: landing", "Press T to take control" if not safety.video_ok
                       else "Video is back. Press T or any flight key to take control")
            else:
                msg = ("Video lost: hovering. Take control?", f"Press T to fly without video. Landing in {math.ceil(safety.countdown())} s")
        if tests and tests.result:
            msg = None   # the test panel is showing the result
        if msg:
            t1, t2 = self.big.render(msg[0], True, INK), self.font.render(msg[1], True, INK)
            bw, bh = max(t1.get_width(), t2.get_width()) + 36, t1.get_height() + t2.get_height() + 26
            box = pygame.Rect(0, 0, bw, bh)
            box.center = (w / 2, h * 0.44)
            pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=10)
            self.screen.blit(t1, t1.get_rect(midtop=(w / 2, box.y + 10)))
            self.screen.blit(t2, t2.get_rect(midtop=(w / 2, box.y + 14 + t1.get_height())))
        # big DISARM button (click it, or press Space)
        self.disarm_box = pygame.Rect(0, 0, 190, 46)
        self.disarm_box.midbottom = (w / 2, h - 44)
        pygame.draw.rect(self.screen, BAD if cc.arm else (170, 170, 166), self.disarm_box, border_radius=10)
        label = self.big.render("DISARM", True, (255, 255, 255))
        self.screen.blit(label, label.get_rect(center=self.disarm_box.center))
        # help line
        self._chip(key_help, 10, h - 32)

    # Which way the take-off pad is and how far, so a drone that has wandered off can be brought
    # back. Up is ahead of the drone's nose. Simulator only: the real drone doesn't know where it is.
    def _pad_pointer(self, x, y, sim):
        box = pygame.Rect(x, y, 85, 85)
        pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=8)
        pygame.draw.rect(self.screen, (200, 196, 186), box, 1, border_radius=8)
        dim, dist = (110, 118, 122), math.hypot(sim.pos[0], sim.pos[1])
        self.screen.blit(self.font.render("sim only", True, dim), (x + 5, y + 3))
        label = self.font.render(f"pad {dist:.0f} m", True, dim)
        self.screen.blit(label, label.get_rect(midbottom=(x + 42.5, y + 83)))
        cx, cy = x + 42.5, y + 44
        if dist < 1:
            pygame.draw.circle(self.screen, INK, (cx, cy), 5)
            return
        a = math.atan2(-sim.pos[1], -sim.pos[0]) - euler(sim.q)[2]   # angle from the nose round to the pad, left is +
        ux, uy = -math.sin(a), -math.cos(a)                          # on screen: left is -x, ahead is up
        tip, tail = (cx + ux * 17, cy + uy * 17), (cx - ux * 17, cy - uy * 17)
        pygame.draw.line(self.screen, INK, tail, tip, 3)
        for side in (1, -1):
            pygame.draw.line(self.screen, INK, tip, (tip[0] - ux * 9 + side * uy * 6, tip[1] - uy * 9 - side * ux * 6), 3)

    # The H panel: every key, plus which drone and settings this is.
    def key_panel(self, keys, about):
        w, h = self.screen.get_size()
        rows = [(self.big.render("Keys", True, INK), None)]
        rows += [(self.font.render(k, True, ACCENT), self.font.render(text, True, INK)) for k, text in keys]
        rows += [(self.font.render(line, True, (110, 118, 122)), None) for line in [""] + about + ["H to close"]]
        bw = max(a.get_width() + (b.get_width() + 80 if b else 0) for a, b in rows) + 40
        bh = sum(a.get_height() + 4 for a, _ in rows) + 24
        box = pygame.Rect(0, 0, bw, bh)
        box.center = (w / 2, h / 2)
        pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=10)
        pygame.draw.rect(self.screen, (200, 196, 186), box, 1, border_radius=10)
        y = box.y + 12
        for a, b in rows:
            self.screen.blit(a, (box.x + 20, y))
            if b:
                self.screen.blit(b, (box.x + 100, y))
            y += a.get_height() + 4

    def _stick(self, x, y, sx, sy, label):
        box = pygame.Rect(x, y, 85, 85)
        pygame.draw.rect(self.screen, (250, 250, 246), box, border_radius=8)
        pygame.draw.rect(self.screen, (200, 196, 186), box, 1, border_radius=8)
        self.screen.blit(self.font.render(label, True, (110, 118, 122)), (x + 5, y + 3))
        pygame.draw.circle(self.screen, INK, (x + 42.5 + sx * 36, y + 42.5 - sy * 36), 8)

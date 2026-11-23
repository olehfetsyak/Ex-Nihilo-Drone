# Physics and flight controller for Drone v.0.0. No graphics here, so it can run in tests or headless.
#
# The flight controller part works like ESP-FC / Betaflight: gyro calibration at power-up, PT1 filters,
# a Mahony attitude estimate, self-levelling (ANGLE mode), a rate PID and a motor mixer with airmode.
# Physics runs at 4 kHz; the flight loop runs at whatever rate the chosen chip can keep up with.
import math
import random
from collections import deque

from .parts import CHIPS, REGULATORS
from ..params import default_params
from ..quat import qconj, qintegrate, qrot, qyaw, euler

G = 9.81
D2R = math.pi / 180
R2D = 180 / math.pi
PHYSICS_HZ = 4000

# Motors: front-right, front-left, rear-right, rear-left (x forward, y left).
# Betaflight default, props in: rear-right and front-left spin clockwise seen from above.
# A prop turns the drone the opposite way to its own spin, so those two have turn = +1.
MOTORS = [(1, -1, -1), (1, 1, 1), (-1, -1, 1), (-1, 1, -1)]  # (sx, sy, turn: +1 = turns the drone counter-clockwise)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def pt1_gain(cutoff_hz, dt):
    rc = 1 / (2 * math.pi * cutoff_hz)
    return dt / (rc + dt)


# Resting voltage of a 1S LiPo at a state of charge from 0 to 1.
def ocv(soc):
    pts = [(0, 3.3), (0.1, 3.5), (0.3, 3.7), (0.5, 3.8), (0.8, 3.95), (1, 4.2)]
    for (a, va), (b, vb) in zip(pts, pts[1:]):
        if soc <= b:
            return va + (vb - va) * (soc - a) / (b - a)
    return 4.2


def sign(x):
    return (x > 0) - (x < 0)


class Sim:
    def __init__(self, params=None, seed=12345):
        self.p = default_params()
        if params:
            self.p.update(params)
        self.seed = seed   # which drone this is: the seed decides the build errors (motor mismatch, board tilt, gyro bias)
        self.rng = random.Random(seed)
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self):
        p, g = self.p, self.rng.gauss
        self.t = 0.0
        self.pos, self.vel = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        self.q, self.w = [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        self.mot, self.cmd = [0.0] * 4, [0.0] * 4
        self.cmd_hist = deque()
        # power
        self.soc = p["start_charge"] / 100
        self.vload, self.vreg, self.current = ocv(self.soc), 3.3, 0.0
        self.fc_on, self.boot_t = True, 0.0
        # flight controller state
        self.armed = self.arm_block = self.warned_arm = self.crashed = False
        self.crash_msg = ""
        self.est = [1.0, 0.0, 0.0, 0.0]
        self.gyro_f, self.gyro_prev, self.d_f = [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.acc_f, self.acc_raw, self.gyro_raw = [0.0, 0.0, G], [0.0, 0.0, G], [0.0] * 3
        self.i_term = [0.0] * 3
        self.cal, self.cal_sum, self.cal_n, self.cal_t = [0.0] * 3, [0.0] * 3, 0, 0.4
        self.bias = [g(0, 1) * p["gyro_bias"] for _ in range(3)]
        # a hand-built drone is never perfect: motors differ a little, and the board isn't mounted dead level
        self.motor_k = [1 + g(0, 1) * p["motor_mismatch"] / 100 for _ in range(4)]
        self.acc_bias = [G * math.sin(g(0, 1) * p["level_error_deg"] * D2R) for _ in range(2)] + [0.0]
        self.next_loop_t, self.last_loop_t, self.loop_load = 0.0, 0.0, 0.0
        # link
        self.rc = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thr": 0.0, "arm": False}
        self.rc_rx = dict(self.rc)
        self.last_pkt, self.pkt_acc, self.pkt_queue = 0.0, 0.0, deque()
        self.failsafe, self.fs_t, self.link_cut = False, 0.0, False
        self.telem, self.telem_t, self.tel_acc, self.tel_queue = self._telemetry(), 0.0, 0.0, deque()
        # world
        self.wind_v, self.gust_state = [0.0] * 3, [0.0] * 3
        self.on_ground = True
        self.events = []

    def event(self, msg, kind="info"):
        self.events.append({"t": self.t, "msg": msg, "kind": kind})
        del self.events[:-30]

    def inertia(self):
        m, L = self.p["mass_g"] / 1000, self.p["arm_mm"] / 1000
        return [0.27 * m * L * L + 4e-6, 0.27 * m * L * L + 4e-6, 0.48 * m * L * L + 6e-6]

    # ------------------------------------------------------- link: laptop <-> bridge <-> drone
    # Flight data the drone sends back: only what it can really know. The IMU gives the lean angles
    # (the flight controller's own estimate), and that's it: no height, no speed, no position.
    def _telemetry(self):
        r, pt, _ = euler(self.est)
        return {"armed": self.armed, "failsafe": self.failsafe, "vbat": self.vload, "roll": r, "pitch": pt}

    def _link_step(self, dt):
        p = self.p
        lost = lambda: self.link_cut or self.rng.random() < p["link_loss"] / 100
        delay = lambda: p["link_delay_ms"] / 1000 * (0.8 + 0.4 * self.rng.random())
        # flight data down
        self.tel_acc += dt
        if self.tel_acc >= 1 / p["telem_hz"]:
            self.tel_acc = 0.0
            if self.fc_on and not lost():
                self.tel_queue.append((self.t + delay(), self._telemetry()))
        while self.tel_queue and self.tel_queue[0][0] <= self.t:
            self.telem, self.telem_t = self.tel_queue.popleft()[1], self.t
        # commands up
        self.pkt_acc += dt
        per = 1 / p["packet_hz"]
        while self.pkt_acc >= per:
            self.pkt_acc -= per
            if not lost():
                self.pkt_queue.append((self.t + delay(), dict(self.rc)))
        while self.pkt_queue and self.pkt_queue[0][0] <= self.t:
            self.rc_rx = self.pkt_queue.popleft()[1]
            self.last_pkt = self.t

    # ------------------------------------------------------- flight controller loop
    def _fc_loop(self, dt):
        p = self.p
        # gyro calibration at power-up: average the still gyro, then subtract it
        if self.cal_t > 0:
            self.cal_t -= dt
            if p["gyro_cal"]:
                for i in range(3):
                    self.cal_sum[i] += self.gyro_raw[i]
                self.cal_n += 1
                if self.cal_t <= 0:
                    self.cal = [s / self.cal_n for s in self.cal_sum]
        g_raw = [self.gyro_raw[i] - self.cal[i] for i in range(3)]

        # filters
        kg, kd, ka = pt1_gain(p["gyro_lpf"], dt), pt1_gain(p["dterm_lpf"], dt), pt1_gain(10, dt)
        for i in range(3):
            self.gyro_f[i] += kg * (g_raw[i] - self.gyro_f[i])
            self.acc_f[i] += ka * (self.acc_raw[i] - self.acc_f[i])

        # Mahony attitude estimate: gyro, nudged toward the accelerometer's "down"
        gyr = [x * D2R for x in self.gyro_f]
        an = math.sqrt(sum(a * a for a in self.acc_f))
        if 0.5 * G < an < 1.5 * G:
            a = [x / an for x in self.acc_f]
            v = qrot(qconj(self.est), [0, 0, 1])
            e = [a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]]
            for i in range(3):
                gyr[i] += p["acc_trust"] * e[i]
        self.est = qintegrate(self.est, gyr, dt)

        # radio + failsafe
        rx, since_rx = self.rc_rx, self.t - self.last_pkt
        if not self.failsafe and self.armed and since_rx > p["failsafe_delay_ms"] / 1000:
            self.failsafe, self.fs_t = True, 0.0
            self.event(f"Control link lost: failsafe ({p['failsafe']})", "bad")
        if self.failsafe and since_rx < 0.05:
            self.failsafe = False
            self.event("Link back")
            if p["failsafe"] == "drop":
                self.arm_block = True
        if self.failsafe:
            self.fs_t += dt
            if p["failsafe"] == "drop" or self.fs_t > p["failsafe_land_s"]:
                if self.armed:
                    self.event("Failsafe disarmed the motors", "bad")
                self.armed, self.arm_block = False, True

        # arming
        er, ep, _ = euler(self.est)
        if not rx["arm"]:
            self.arm_block = False
        if not self.armed and rx["arm"] and not self.arm_block and not self.failsafe and not self.crashed and self.cal_t <= 0:
            thr_low = rx["thr"] < 0.06
            if thr_low and abs(er) < 25 * D2R and abs(ep) < 25 * D2R:
                self.armed, self.i_term = True, [0.0] * 3
                self.event("Armed", "good")
            elif not self.warned_arm:
                self.event("Can't arm: drone is tilted" if thr_low else "Can't arm: throttle must be low", "bad")
                self.warned_arm = self.arm_block = True
        if self.armed and not rx["arm"] and not self.failsafe:
            self.armed = False
            self.event("Disarmed")
        if not rx["arm"]:
            self.warned_arm = False

        # setpoints (deg/s): the sticks ask for a lean angle, and the drone levels itself when they're centred
        roll, pitch, yaw, thr_stick = rx["roll"], rx["pitch"], rx["yaw"], rx["thr"]
        if self.failsafe:
            roll = pitch = yaw = 0.0
        sp = [clamp(p["angle_gain"] * (roll * p["max_angle"] - er * R2D), -p["max_rate"], p["max_rate"]),
              clamp(p["angle_gain"] * (pitch * p["max_angle"] - ep * R2D), -p["max_rate"], p["max_rate"]),
              -yaw * p["yaw_rate"]]  # stick right = nose right = negative yaw rate (FLU)

        # throttle: straight through. In failsafe the drone can't see the ground, so like Betaflight it
        # levels out, holds one fixed throttle a bit under hover, and cuts the motors after a set time.
        thr = p["failsafe_throttle"] if self.failsafe and p["failsafe"] == "land" else thr_stick

        # rate PID (Betaflight-style scaling)
        gains = [(p["p_roll"], p["i_roll"], p["d_roll"])] * 2 + [(p["p_yaw"], p["i_yaw"], 0)]
        out = [0.0, 0.0, 0.0]
        saturated = any(c >= 0.999 or c <= 0.051 for c in self.cmd)
        for i in range(3):
            e = sp[i] - self.gyro_f[i]
            d_raw = (self.gyro_f[i] - self.gyro_prev[i]) / dt
            self.gyro_prev[i] = self.gyro_f[i]
            self.d_f[i] += kd * (d_raw - self.d_f[i])
            if self.armed and not (saturated and sign(e) == sign(self.i_term[i])) and thr > 0.05:
                self.i_term[i] = clamp(self.i_term[i] + gains[i][1] * 1e-5 * e * dt, -0.3, 0.3)
            if not self.armed:
                self.i_term[i] = 0.0
            out[i] = gains[i][0] * 1e-5 * e + self.i_term[i] - gains[i][2] * 1e-7 * self.d_f[i]
        out[2] = clamp(out[2], -0.3, 0.3)

        # mixer with airmode
        base = 0.05 + 0.95 * thr
        m = [base + out[0] * sy + out[1] * -sx + out[2] * spin for sx, sy, spin in MOTORS]
        if p["airmode"] or thr > 0.05:
            hi, lo = max(m), min(m)
            if hi > 1:
                m = [x - (hi - 1) for x in m]
            elif lo < 0.05:
                m = [x + (0.05 - lo) for x in m]
        self.cmd = [clamp(x, 0.05, 1) for x in m] if self.armed else [0.0] * 4
        if self.armed and thr <= 0 and not p["airmode"]:
            self.cmd = [0.05] * 4

    # ------------------------------------------------------- physics step
    def step(self, dt=1 / PHYSICS_HZ):
        p, gauss = self.p, self.rng.gauss
        I, m = self.inertia(), p["mass_g"] / 1000
        L = p["arm_mm"] / 1000 * math.sqrt(0.5)
        self.t += dt
        self._link_step(dt)

        # power: battery sag, regulator, brownout
        v_norm = clamp(self.vload / 4.2, 0, 1.2)
        t_max = p["motor_thrust_g"] / 1000 * G * v_norm * v_norm
        thrusts = [t_max * s * s * k for s, k in zip(self.mot, self.motor_k)]
        self.current = sum(24 * max(T, 0) ** 1.5 for T in thrusts) + 0.25
        self.soc = max(0.0, self.soc - self.current * dt / 3600 / (p["battery_mah"] / 1000))
        self.vload = max(0.0, ocv(self.soc) - self.current * p["rint_mohm"] / 1000)
        reg = REGULATORS[p["regulator"]]
        if reg["min_in"]:
            self.vreg = 3.3 if self.vload >= reg["min_in"] else self.vload
        else:
            self.vreg = min(3.3, self.vload - reg["dropout"])
        if self.fc_on and self.vreg < p["brownout_v"]:
            self.fc_on, self.boot_t = False, 0.6
            if self.armed:
                self.event(f"Brownout: regulator output fell to {self.vreg:.2f} V and the flight controller rebooted", "bad")
            self.armed, self.arm_block, self.cmd = False, True, [0.0] * 4
        if not self.fc_on:
            self.boot_t -= dt
            self.cmd = [0.0] * 4
            if self.boot_t <= 0 and self.vreg >= p["brownout_v"] + 0.05:
                self.fc_on, self.est, self.cal_t = True, [1.0, 0.0, 0.0, 0.0], 0.4
                self.cal_sum, self.cal_n, self.gyro_f = [0.0] * 3, 0, [0.0] * 3
                self.event("Flight controller rebooted. Press Space, then Enter to re-arm")

        # gyro with noise, bias and motor vibration
        thr_avg = sum(self.mot) / 4
        vib, ph = p["vibration"] * 60 * thr_avg, 2 * math.pi * p["vib_freq"] * self.t
        for i in range(3):
            self.gyro_raw[i] = (self.w[i] * R2D + self.bias[i] + gauss(0, 1) * p["gyro_noise"]
                                + vib * math.sin(ph + i * 2.1) * (0.4 if i == 2 else 1))

        # flight loop scheduling: a slow chip can't run the loop as fast as asked
        chip = CHIPS[p["chip"]]
        cost, want_per = chip["loop_cost_us"] / 1e6, 1 / p["loop_hz"]
        self.loop_load = cost / want_per + chip["radio_share"]
        eff_per = max(want_per, cost / (1 - min(0.9, chip["radio_share"])))
        if self.fc_on and self.t >= self.next_loop_t - 1e-9:
            jitter = eff_per * 0.3 * self.rng.random() if self.loop_load > 0.85 else 0.0
            self.next_loop_t = self.t + eff_per + jitter
            # like a real flight controller, the loop uses the time that really passed since it last ran
            loop_dt, self.last_loop_t = min(0.01, self.t - self.last_loop_t), self.t
            self._fc_loop(loop_dt)

        # ESC delay, then motor spin-up lag
        n_delay = max(1, round(p["esc_delay_ms"] / 1000 / dt))
        self.cmd_hist.append(list(self.cmd))
        while len(self.cmd_hist) > n_delay:
            self.cmd_hist.popleft()
        cmd_d = self.cmd_hist[0]
        km = dt / (p["motor_tau_ms"] / 1000 + dt)
        for i in range(4):
            self.mot[i] += km * (cmd_d[i] - self.mot[i])

        # forces and torques
        t_sum = sum(thrusts)
        tq = [0.0, 0.0, 0.0]
        for (sx, sy, spin), T in zip(MOTORS, thrusts):
            tq[0] += sy * L * T
            tq[1] += -sx * L * T
            tq[2] += spin * p["yaw_coef"] * T
        tq[0] += -m * G * p["cg_offset_mm"] / 1000 * math.cos(euler(self.q)[0])  # CG to the right pulls right side down
        for i in range(3):
            tq[i] -= 3e-6 * self.w[i]

        # wind and gusts, drag
        # gusts wander around zero, typically p["gust"] m/s strong and lasting about 0.8 s
        gk = dt / 0.8
        for i in range(3):
            self.gust_state[i] += -gk * self.gust_state[i] + p["gust"] * math.sqrt(2 * gk) * gauss(0, 1)
        wd = p["wind_dir"] * D2R
        self.wind_v = [math.cos(wd) * p["wind"] + self.gust_state[0],
                       math.sin(wd) * p["wind"] + self.gust_state[1], self.gust_state[2] * 0.3]
        rel = [self.vel[i] - self.wind_v[i] for i in range(3)]
        rs = math.sqrt(sum(v * v for v in rel))
        # two parts: one grows with speed (the spinning props), one with speed squared (the body; air is 1.2 kg/m3)
        drag = [-(p["drag_linear"] * v + 0.5 * 1.2 * p["drag_area_cm2"] / 1e4 * v * rs) for v in rel]
        fb = qrot(self.q, [0, 0, t_sum])
        acc = [(fb[0] + drag[0]) / m, (fb[1] + drag[1]) / m, (fb[2] + drag[2]) / m - G]

        # integrate
        w = self.w
        wdot = [tq[0] / I[0] - (I[2] - I[1]) / I[0] * w[1] * w[2],
                tq[1] / I[1] - (I[0] - I[2]) / I[1] * w[0] * w[2],
                tq[2] / I[2]]
        for i in range(3):
            self.vel[i] += acc[i] * dt
            self.pos[i] += self.vel[i] * dt
            self.w[i] += wdot[i] * dt
        self.q = qintegrate(self.q, self.w, dt)

        # accelerometer: specific force in the body frame
        sf = qrot(qconj(self.q), [acc[0], acc[1], acc[2] + G])
        self.acc_raw = [x + b + gauss(0, 1) * p["acc_noise"] * G + vib * 0.01 * math.sin(ph * 1.01)
                        for x, b in zip(sf, self.acc_bias)]

        # ground contact and crash check
        self.on_ground = False
        if self.pos[2] <= 0:
            r, pi, y = euler(self.q)
            tilt_deg = max(abs(r), abs(pi)) * R2D
            spd = math.sqrt(sum(v * v for v in self.vel))
            if not self.crashed and (self.vel[2] < -3 or (tilt_deg > 60 and spd > 1.5) or spd > 6):
                self.crashed, self.armed = True, False
                self.crash_msg = f"Crashed into the ground at {spd:.1f} m/s"
                self.event(self.crash_msg, "bad")
            self.pos[2], self.on_ground = 0.0, True
            self.vel[2] = max(self.vel[2], 0.0)
            self.vel[0] *= 0.9
            self.vel[1] *= 0.9
            if t_sum < m * G * 0.98:
                self.q, self.w = qyaw(y), [0.0, 0.0, self.w[2] * 0.9]

    def run(self, seconds):
        for _ in range(round(seconds * PHYSICS_HZ)):
            self.step()

    def collide(self, boxes):
        x, y, z = self.pos
        r = 0.06
        for b in boxes:
            if (not self.crashed and b["x"] - b["sx"] / 2 - r < x < b["x"] + b["sx"] / 2 + r
                    and b["y"] - b["sy"] / 2 - r < y < b["y"] + b["sy"] / 2 + r and z < b["sz"] + r):
                self.crashed, self.armed = True, False
                self.crash_msg = "Hit an obstacle"
                self.event("Crashed into an obstacle", "bad")
                self.vel = [0.0, 0.0, min(self.vel[2], 0.0)]

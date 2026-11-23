# Flight checks that run without a window:  python tests/test_sim.py
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drone_sim.command_center import CommandCenter
from drone_sim.flight_tests import FlightTests
from drone_sim.quat import euler
from drone_sim.hardware.drone import R2D, Sim
from drone_sim.world import BOXES

FRAME = 1 / 60


# Hold some keys for a while, one Command Center update per frame like the real program.
def fly(cc, held, seconds, ft=None, until=None):
    for _ in range(round(seconds / FRAME)):
        cc.update(set(held), FRAME)
        cc.advance(FRAME)
        cc.sim.collide(BOXES)
        if ft:
            ft.update(FRAME)
        if until and until():
            return


def armed_on_ground(**params):
    sim = Sim(params)
    cc = CommandCenter(sim)
    fly(cc, [], 0.6)           # gyro calibration
    cc.arm = True
    fly(cc, [], 0.3)
    assert sim.armed
    return sim, cc


def take_off(cc):
    cc.pilot.key_down("w")
    fly(cc, ["w"], 1.0)
    fly(cc, [], 3.0)


def tilt_deg(sim):
    r, p, _ = euler(sim.q)
    return max(abs(r), abs(p)) * R2D


def ground_speed(sim):
    return math.hypot(sim.vel[0], sim.vel[1])


# ------------------------------------------------------------------ flying with the keyboard
def test_arms_and_stays_down_hands_off():
    sim, cc = armed_on_ground()
    fly(cc, [], 1.0)
    assert sim.pos[2] == 0 and sim.rc["thr"] == 0


def test_w_takes_off_and_hands_off_roughly_hovers():
    sim, cc = armed_on_ground()
    take_off(cc)
    assert sim.pos[2] > 1
    fly(cc, [], 3.0)
    assert abs(sim.vel[2]) < 0.4, "the hover guess is close on the default drone, so it barely climbs or sinks"
    assert 0.55 < sim.rc["thr"] < 0.68, "hover throttle is about 60%"


def test_leaning_moves_right_and_levels_out():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.pilot.key_down("right")
    fly(cc, ["right"], 1.0)
    assert euler(sim.q)[0] * R2D > 15, "right arrow leans right side down"
    assert sim.vel[1] < -1, "and moves right (negative y)"
    fly(cc, [], 2.0)
    assert abs(euler(sim.q)[0] * R2D) < 4, "levels out after letting go"


def test_yaw_d_turns_right():
    sim, cc = armed_on_ground()
    take_off(cc)
    y0 = euler(sim.q)[2]
    cc.pilot.key_down("d")
    fly(cc, ["d"], 0.5)
    assert euler(sim.q)[2] < y0 - 0.3


def test_s_lands_but_motors_run_until_disarmed():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.pilot.key_down("s")
    fly(cc, ["s"], 12.0)
    assert sim.pos[2] == 0 and not sim.crashed
    assert sim.armed and sim.rc["thr"] > 0.3, "the drone can't tell it has landed"
    cc.disarm()
    fly(cc, [], 0.2)
    assert not sim.armed and sim.cmd == [0.0] * 4


def test_heavier_build_sinks_until_trimmed():
    sim, cc = armed_on_ground(mass_g=60)   # the laptop's hover guess is for 55 g
    cc.pilot.key_down("w")
    fly(cc, ["w"], 2.0)
    fly(cc, [], 2.5)
    assert sim.vel[2] < -0.3 or sim.pos[2] == 0, "hands off, it sinks back down"
    for _ in range(3):
        cc.pilot.key_down("e")
    cc.pilot.key_down("w")
    fly(cc, ["w"], 1.5)
    fly(cc, [], 3.0)
    assert abs(sim.vel[2]) < 0.35 and sim.pos[2] > 0, "three taps of E and it holds"


def test_hand_built_drone_drifts():
    sim, cc = armed_on_ground()
    take_off(cc)
    fly(cc, [], 5.0)
    assert 0.03 < ground_speed(sim) < 0.8, "never perfectly still, but slow"


def test_disarm_button_stops_the_motors():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.disarm()
    fly(cc, [], 0.2)
    assert not sim.armed and sim.cmd == [0.0] * 4


def test_arm_key_never_disarms():
    sim = Sim()
    cc = CommandCenter(sim)
    fly(cc, [], 0.7)
    cc.arm_drone()
    fly(cc, [], 0.3)
    cc.arm_drone()
    fly(cc, [], 0.3)
    assert sim.armed, "a second press of the arm key changes nothing"
    cc.disarm()
    fly(cc, [], 0.3)
    assert not sim.armed


def test_cant_arm_with_throttle_up():
    sim = Sim()
    sim.run(0.6)
    sim.rc.update(arm=True, thr=0.5)
    sim.run(0.2)
    assert not sim.armed and any("throttle must be low" in e["msg"] for e in sim.events)


# ------------------------------------------------------------------ drone hardware
def test_ams1117_browns_out_on_takeoff():
    sim, cc = armed_on_ground(regulator="ams1117")
    cc.pilot.key_down("w")
    fly(cc, ["w"], 2.0)
    assert any("Brownout" in e["msg"] for e in sim.events)


def test_chip_load():
    for chip, hz, overloaded in [("c3", 1000, False), ("c3", 4000, True), ("s3", 4000, False)]:
        sim = Sim({"chip": chip, "loop_hz": hz})
        sim.step()
        assert (sim.loop_load > 1) == overloaded, (chip, hz, sim.loop_load)


def test_hard_landing_crashes():
    sim = Sim()
    sim.pos, sim.vel = [0, 0, 3], [0, 0, -5]
    sim.run(1.0)
    assert sim.crashed


def test_obstacle_crash():
    sim = Sim()
    b = BOXES[0]
    sim.pos = [b["x"], b["y"], 0.3]
    sim.collide(BOXES)
    assert sim.crashed


def test_wind_pushes_the_drone():
    sim, cc = armed_on_ground(wind=3.0)
    take_off(cc)
    assert abs(sim.pos[1]) > 1


def test_gusts_are_as_strong_as_the_setting():
    sim = Sim({"gust": 1.0})
    speeds = []
    for _ in range(40000):   # 10 s
        sim.step()
        speeds.append(abs(sim.wind_v[0]))
    assert 0.5 < sum(speeds) / len(speeds) < 1.1, "gust=1 means gusts of about 1 m/s"
    calm = Sim()
    calm.run(1.0)
    assert calm.wind_v == [0.0, 0.0, 0.0]


def test_smaller_battery_drains_faster():
    used = []
    for mah in (300, 150):
        sim, cc = armed_on_ground(battery_mah=mah)
        take_off(cc)
        used.append(1 - sim.soc)
    assert 1.8 < used[1] / used[0] < 2.2


# ------------------------------------------------------------------ link and drone failsafe
def test_flight_data_is_only_what_the_drone_can_sense():
    sim, cc = armed_on_ground()
    take_off(cc)
    assert set(sim.telem) == {"armed", "failsafe", "vbat", "roll", "pitch"}
    assert sim.telem["armed"] and sim.t - sim.telem_t < 0.1


def test_failsafe_land_comes_down_then_cuts_motors():
    sim, cc = armed_on_ground()
    take_off(cc)
    z = sim.pos[2]
    sim.link_cut = True
    fly(cc, [], 1.5)
    assert sim.failsafe and sim.armed and sim.pos[2] < z
    assert not cc.safety.link_ok and cc.safety.reason == "link lost", "the laptop notices too"
    fly(cc, [], sim.p["failsafe_land_s"])
    assert not sim.armed, "motors cut after the set time"
    fly(cc, [], 2.0)
    assert sim.pos[2] == 0 and not sim.crashed


def test_failsafe_drop_cuts_motors():
    sim, cc = armed_on_ground(failsafe="drop")
    take_off(cc)
    sim.link_cut = True
    fly(cc, [], 0.6)
    assert not sim.armed and sim.cmd == [0.0] * 4


def test_link_back_gives_control_back():
    sim, cc = armed_on_ground()
    cc.pilot.key_down("w")
    fly(cc, ["w"], 2.5)
    sim.link_cut = True
    fly(cc, [], 1.0)
    assert sim.failsafe
    sim.link_cut = False
    fly(cc, [], 3.0)
    assert sim.armed and not sim.failsafe and cc.safety.source == "keyboard"
    assert abs(sim.vel[2]) < 0.4 and sim.pos[2] > 0, "hovering again"


# ------------------------------------------------------------------ safety layer
def test_safety_tilt_limit():
    sim, cc = armed_on_ground(safety_max_tilt=15, kb_strength=1.0)
    take_off(cc)
    cc.pilot.key_down("up")
    fly(cc, ["up"], 1.0)
    assert 8 < tilt_deg(sim) < 19, tilt_deg(sim)


def test_video_lost_levels_out_and_hovers():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.pilot.key_down("up")
    fly(cc, ["up"], 1.5)
    fast = ground_speed(sim)
    assert fast > 2
    cc.video.cut = True
    x = sim.pos[0]
    fly(cc, [], 1.5)
    assert cc.safety.source == "safety" and cc.safety.reason == "video lost"
    assert tilt_deg(sim) < 6, "level"
    assert ground_speed(sim) > 0.8, "still coasting: the drone can't measure its speed to brake"
    fly(cc, [], 2.5)
    assert ground_speed(sim) < fast / 2 and sim.pos[0] > x + 2.5, "only drag slows it, metres later"
    assert sim.pos[2] > 0.5 and sim.armed


def test_video_lost_and_nobody_answers_so_it_lands_and_stays_down():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.video.cut = True
    fly(cc, [], 4.0)
    assert not cc.safety.landing and sim.pos[2] > 0.5, "hovers first, waiting for the pilot"
    fly(cc, [], 2.0)
    assert cc.safety.landing, "then lands"
    fly(cc, [], 15.0)
    assert sim.pos[2] == 0 and not sim.crashed
    cc.video.cut = False
    fly(cc, [], 3.0)
    assert sim.pos[2] == 0, "doesn't lift off again when the video returns"
    assert sim.armed, "someone still has to disarm it"


def test_video_lost_locks_the_keys_until_the_pilot_takes_control():
    sim, cc = armed_on_ground()
    take_off(cc)
    cc.video.cut = True
    fly(cc, [], 1.0)
    cc.pilot.key_down("right")
    fly(cc, ["right"], 1.0)
    assert cc.safety.source == "safety" and tilt_deg(sim) < 6, "keys do nothing until the pilot answers"
    cc.take_control()
    fly(cc, ["right"], 1.0)
    assert cc.safety.source == "keyboard" and euler(sim.q)[0] * R2D > 15, "the pilot always overrides"
    fly(cc, [], 6.0)
    assert not cc.safety.landing and sim.pos[2] > 0.3, "and it doesn't land itself after that"
    cc.video.cut = False
    fly(cc, [], 0.5)
    assert not cc.safety.takeover, "video back: next time it asks again"


def test_landing_hands_back_on_the_first_key_after_video_returns():
    sim, cc = armed_on_ground()
    cc.pilot.key_down("w")
    fly(cc, ["w"], 2.5)
    cc.video.cut = True
    fly(cc, [], 6.0)
    cc.video.cut = False
    fly(cc, [], 1.0)
    assert cc.safety.source == "safety" and cc.safety.landing, "still landing"
    cc.pilot.key_down("w")
    fly(cc, ["w"], 0.5)
    assert cc.safety.source == "keyboard"


def test_no_arming_or_take_off_without_video():
    sim = Sim()
    cc = CommandCenter(sim)
    fly(cc, [], 0.7)
    cc.video.cut = True
    fly(cc, [], 0.8)
    cc.arm_drone()
    assert not cc.arm
    cc.video.cut = False
    fly(cc, [], 0.3)
    cc.arm_drone()
    fly(cc, [], 0.3)
    assert sim.armed
    cc.video.cut = True
    fly(cc, [], 0.8)
    cc.pilot.key_down("w")
    fly(cc, ["w"], 2.0)
    assert sim.pos[2] == 0 and sim.rc["thr"] == 0


def test_slow_video_still_counts_as_video():
    sim, cc = armed_on_ground(video_delay_ms=700)
    cc.pilot.key_down("w")
    fly(cc, ["w"], 1.5)
    assert cc.safety.video_ok and sim.pos[2] > 0.5, "late frames that keep coming are not a lost video"
    cc.video.cut = True
    fly(cc, [], 1.0)
    assert not cc.safety.video_ok


def test_low_battery_warning():
    sim, cc = armed_on_ground(start_charge=15)
    assert not cc.safety.low_battery
    take_off(cc)
    assert cc.safety.low_battery


# ------------------------------------------------------------------ flight tests (the number keys)
def start_test(n, **params):
    sim = Sim(params)
    cc = CommandCenter(sim)
    ft = FlightTests(cc)
    ft.start(n)
    fly(cc, [], 0.7, ft)
    cc.arm_drone()
    fly(cc, [], 0.3, ft)
    cc.pilot.key_down("w")
    return sim, cc, ft


def land_and_disarm(cc, ft):
    cc.pilot.key_down("s")
    fly(cc, ["s"], 15, ft, until=lambda: cc.sim.on_ground)
    cc.disarm()
    fly(cc, [], 0.3, ft)


# Taps toward a spot (over the pad unless told otherwise), longer the further off it is, like a pilot
# feathering the keys. Uses the true position, as a pilot's eyes would. The drone never turns in
# these tests, so forward is +x and left is +y.
def nudge(sim, z, x=0.0, y=0.0):
    keys, phase = [], (sim.t % 0.3) / 0.3
    for want, plus, minus in ((1.2 * (z - sim.pos[2]) - 0.9 * sim.vel[2], "w", "s"),
                              (0.5 * (x - sim.pos[0]) - 0.7 * sim.vel[0], "up", "down"),
                              (0.5 * (y - sim.pos[1]) - 0.7 * sim.vel[1], "left", "right")):
        if abs(want) > 0.12 and phase < abs(want):
            keys.append(plus if want > 0 else minus)
    return keys


def test_flight_test_hover_only_counts_a_real_hover():
    sim, cc, ft = start_test(1)
    fly(cc, ["w"], 5, ft, until=lambda: sim.pos[2] > 1.0)
    assert ft.hold == 0 and "too fast" in ft.note, "climbing through the rings is not hovering"
    for _ in range(round(40 / FRAME)):
        fly(cc, nudge(sim, 1.25), FRAME, ft)
        if ft.result:
            break
    assert ft.result and ft.result[0], ft.result
    assert "% throttle" in ft.result[1]


def test_flight_test_weak_battery_depends_on_the_regulator():
    sim, cc, ft = start_test(2)
    fly(cc, ["w"], 10, ft, until=lambda: ft.stage == 1)
    land_and_disarm(cc, ft)
    assert ft.result and ft.result[0], ft.result
    sim, cc, ft = start_test(2, regulator="ams1117")
    fly(cc, ["w"], 3, ft, until=lambda: ft.result)
    assert ft.result and not ft.result[0] and "Brownout" in ft.result[1]


def test_flight_test_link_drop():
    sim, cc, ft = start_test(5)
    fly(cc, ["w"], 10, ft, until=lambda: sim.pos[2] > 4)
    assert ft.stage == 0, "still climbing fast: the link isn't cut until it has settled"
    fly(cc, [], 10, ft, until=lambda: ft.stage == 1)
    assert sim.link_cut
    fly(cc, [], 5, ft, until=lambda: ft.stage == 2)
    assert not sim.link_cut
    land_and_disarm(cc, ft)
    assert ft.result and ft.result[0], ft.result


def test_flight_test_video_drop():
    sim, cc, ft = start_test(6)
    fly(cc, ["w"], 1.0, ft)
    cc.pilot.key_down("up")
    fly(cc, ["up"], 10, ft, until=lambda: ft.stage == 1)
    assert math.hypot(sim.vel[0], sim.vel[1]) > 2, "the video only drops once it is really moving"
    x = sim.pos[0]
    fly(cc, ["down"], 40, ft, until=lambda: sim.on_ground)
    assert sim.pos[0] > x + 4, "keys locked: it coasts on, however hard you pull back"
    cc.disarm()
    fly(cc, [], 0.3, ft)
    assert ft.result and ft.result[0], ft.result


def test_flight_test_lag_needs_a_landing_inside_the_ring():
    sim, cc, ft = start_test(7)
    fly(cc, ["w"], 1.0, ft)
    land_and_disarm(cc, ft)
    assert ft.result and not ft.result[0] and "from the middle" in ft.result[1], "landing on the pad is not in the ring"
    fly_to_the_ring_and_land(7)


# Flies test n to its ring with the tapping pilot, lands in it and disarms. Must pass.
def fly_to_the_ring_and_land(n):
    sim, cc, ft = start_test(n)
    x, y, _, r = ft.rings()[0]
    over = lambda: math.hypot(sim.pos[0] - x, sim.pos[1] - y) < 0.5 and ground_speed(sim) < 0.5
    for _ in range(round(60 / FRAME)):
        fly(cc, nudge(sim, 1.5, x, y), FRAME, ft)
        if over():
            break
    assert over(), f"test {n}: got over the ring"
    for _ in range(round(20 / FRAME)):
        fly(cc, nudge(sim, -1.0, x, y), FRAME, ft)
        if sim.on_ground:
            break
    cc.disarm()
    fly(cc, [], 0.3, ft)
    assert ft.result and ft.result[0], ft.result


def test_flight_test_lost_packets_can_be_flown():
    fly_to_the_ring_and_land(8)


def test_flight_tests_crooked_build_and_breeze_can_be_held():
    for n in (3, 4):
        sim, cc, ft = start_test(n)
        for _ in range(round(40 / FRAME)):
            fly(cc, nudge(sim, 1.5), FRAME, ft)
            if ft.result:
                break
        assert ft.result and ft.result[0], (n, ft.result)
    sim, cc, ft = start_test(4)
    fly(cc, ["w"], 2.0, ft)
    fly(cc, [], 20, ft, until=lambda: abs(sim.pos[1]) > 3.5)
    assert abs(sim.pos[1]) > 3.5 and ft.hold == 0 and "outside the ring" in ft.note, "hands off, the breeze carries it out"


def test_settings_from_the_command_line():
    from run_sim import parse_settings
    assert parse_settings(["mass_g=60", "regulator=ams1117", "wind=2", "airmode=off"]) == \
        {"mass_g": 60, "regulator": "ams1117", "wind": 2.0, "airmode": False}
    for bad in (["mass=60"], ["regulator=magic"], ["mass_g=heavy"], ["wind"],
                ["video_fps=0"], ["mass_g=-5"], ["wind=nan"], ["telem_hz=2"], ["video_delay_ms=2000"]):
        try:
            parse_settings(bad)
            assert False, bad
        except SystemExit:
            pass


def test_every_setting_has_limits_and_the_defaults_are_inside_them():
    from drone_sim.params import LIMITS, default_params
    from run_sim import CHOICES
    for name, value in default_params().items():
        if isinstance(value, bool):
            continue
        if isinstance(value, str):
            assert value in CHOICES[name], name
        else:
            assert LIMITS[name][0] <= value <= LIMITS[name][1], name


def test_flies_at_both_ends_of_the_limits_without_an_error():
    from drone_sim.params import LIMITS
    for name, ends in LIMITS.items():
        for value in ends:
            sim = Sim({name: value})
            cc = CommandCenter(sim)
            fly(cc, [], 0.5)
            cc.arm = True
            cc.pilot.key_down("w")
            fly(cc, ["w"], 0.3)
            cc.video.age(sim)
            assert all(math.isfinite(v) for v in sim.pos + sim.vel), (name, value)


# ------------------------------------------------------------------ which drone, stuttering video, the window
def test_seed_picks_the_drone():
    a, b, c = Sim(seed=7), Sim(seed=7), Sim(seed=8)
    assert a.motor_k == b.motor_k and a.acc_bias == b.acc_bias, "same seed, same build errors"
    assert a.motor_k != c.motor_k, "another seed is another drone"
    cc = CommandCenter(a)
    ft = FlightTests(cc)
    first = list(a.motor_k)
    cc.pilot.key_down("w")
    fly(cc, ["w"], 1.0, ft)
    ft.start(0)
    assert a.motor_k == first, "a reset in free flight gives back the same drone"
    ft.start(3)
    crooked = list(a.motor_k)
    ft.start(0)
    ft.start(3)
    assert a.motor_k == crooked and crooked != first, "a flight test always uses its own drone"


def test_stuttering_video():
    sim, cc = armed_on_ground(video_loss=50)
    take_off(cc)
    ages, lost = [], 0
    for _ in range(round(20 / FRAME)):
        fly(cc, [], FRAME)
        ages.append(cc.video.age(sim))
        lost += not cc.safety.video_ok
    assert max(ages) > 0.15, "frames go missing, so the picture freezes for a moment"
    assert lost < 0.1 * len(ages), "but half the frames still arriving is a working video nearly all the time"
    assert sim.pos[2] > 0.3 and not sim.crashed
    smooth, _ = armed_on_ground()
    stutter, _ = armed_on_ground(video_loss=50)
    assert smooth.motor_k == stutter.motor_k and smooth.pos == stutter.pos, "lost frames don't change the drone itself"
    sim = Sim({"video_loss": 100})
    cc = CommandCenter(sim)
    fly(cc, [], 1.5)
    cc.arm_drone()
    assert not cc.arm and not cc.safety.video_ok, "no frame ever arrives: that is no video, so no arming"


def test_window_draws_everything_without_a_screen():
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    import run_sim
    from drone_sim.view import View
    pygame.init()
    try:
        sim, cc = armed_on_ground()
        ft = FlightTests(cc)
        for size in ((1100, 700), (320, 200), (1, 1)):
            view = View(pygame.display.set_mode(size))
            for n in (0, 1, 6):
                ft.start(n)
                fly(cc, [], 0.2, ft)
                for cam in ("chase", "fpv", "ground"):
                    view.cam = cam
                    for pos in ([0.0, 0.0, 0.0], [0.4, 0.2, 1.5], [150.0, -90.0, 30.0]):
                        sim.pos = list(pos)
                        view.render(sim, FRAME, cc.video, ft.rings())
                        view.hud(sim, False, run_sim.KEY_HELP, cc, ft)
                        view.key_panel(run_sim.ALL_KEYS, ["Drone 1", "Changed settings: none"])
            cc.video.cut = True
            fly(cc, [], 1.0, ft)
            view.render(sim, FRAME, cc.video, ft.rings())
            view.hud(sim, True, run_sim.KEY_HELP, cc, ft)
    finally:
        pygame.quit()


# ------------------------------------------------------------------ tuning report
def test_tuning_report_tells_good_gains_from_bad():
    from drone_sim.hardware.tuning import advice, measure
    good = measure()
    assert abs(good["hover_setting"] - 0.6) < 0.03, "finds the hover throttle the default drone was set up with"
    assert good["wobble"] < 2 and good["roll_over"] < 10 and good["roll_rise"] < 0.6
    assert abs(good["roll_final"] - good["roll_target"]) < 3, "leans to what was asked"
    assert advice(good) == ["Nothing stands out."]
    soft = measure({"angle_gain": 2})
    assert soft["roll_rise"] > good["roll_rise"] + 0.15 and any("slow to lean" in n for n in advice(soft))
    shaky = measure({"esc_delay_ms": 30})
    assert shaky["wobble"] > 50 and any("shakes" in n for n in advice(shaky))
    assert "shakes" in measure({"p_roll": 180})["failed"]
    assert "browned out" in measure({"regulator": "ams1117"})["failed"]
    heavy = measure({"mass_g": 65})
    assert heavy["hover_setting"] > good["hover_setting"] + 0.03, "a heavier build needs more hover throttle"


def test_attitude_estimate_stays_true_through_a_long_flight():
    sim, cc = armed_on_ground(level_error_deg=0, motor_mismatch=0)
    take_off(cc)
    worst = 0.0
    for _ in range(12):   # lean hard one way then the other, for 12 s
        for key in ("right", "left"):
            cc.pilot.key_down(key)
            fly(cc, [key], 0.5)
            worst = max(worst, abs(euler(sim.q)[0] - euler(sim.est)[0]) * R2D)
    assert worst < 4, f"the flight controller's idea of its lean was {worst:.1f} deg off"


def test_flight_test_crash_fails_and_settings_go_back():
    sim, cc, ft = start_test(4, mass_g=58)
    assert sim.p["wind"] == 2.0 and sim.p["mass_g"] == 58
    fly(cc, ["w"], 2.5, ft)
    cc.disarm()
    fly(cc, [], 3, ft, until=lambda: ft.result)
    assert ft.result and not ft.result[0]
    ft.start(0)
    assert sim.p["wind"] == 0.0 and sim.p["mass_g"] == 58 and not sim.crashed


if __name__ == "__main__":
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print("ok  ", name)
        except AssertionError as e:
            failed += 1
            print("FAIL", name, e)
    print(f"{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)

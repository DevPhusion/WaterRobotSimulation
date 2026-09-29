import atexit
import csv
import datetime
import importlib
import json
import math
import os
import random
import re
import shutil
import signal
import subprocess
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

WORLD_NAME = "water_test"
MODEL_NAME = "test_boat"
LEFT_JOINT = "left_thruster_joint"
RIGHT_JOINT = "right_thruster_joint"
BODY_YAW_OFFSET = 0.0

START_POINT = (0.0, 0.0)
END_POINT = (20.0, 0.0)
GOAL_RADIUS = 1.0

_dx = END_POINT[0] - START_POINT[0]
_dy = END_POINT[1] - START_POINT[1]
LINE_LENGTH = math.hypot(_dx, _dy)
LINE_HEADING = math.atan2(_dy, _dx)
LINE_DIR = (_dx / LINE_LENGTH, _dy / LINE_LENGTH)

MAX_THRUST = 6.0
BASE_THRUST_DEFAULT = 2.5
KP_DEFAULT = 4.0
KI_DEFAULT = 0.0
KD_DEFAULT = 0.8
KXTE_DEFAULT = 0.0
KXTE_DOT_DEFAULT = 0.0
KXTE_DDOT_DEFAULT = 0.0
DRIFT_FILTER_TAU_DEFAULT = 0.5  # s; larger = smoother but more lag

TICK_S = 0.05  # ~20 Hz control loop

USE_WSL = False

WAVE_TEMPLATE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf.template")
WAVE_REAL_SDF_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf")
WORLD_FILE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf")

POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
LEFT_TOPIC = f"/model/{MODEL_NAME}/joint/{LEFT_JOINT}/cmd_thrust"
RIGHT_TOPIC = f"/model/{MODEL_NAME}/joint/{RIGHT_JOINT}/cmd_thrust"
CURRENT_TOPIC = "/ocean_current"

ENV_PRESETS = {
    "lake": {
        "label": "Lake",
        "current_speed": (0.05, 0.30),   # m/s
        "wind_speed": (0.8, 2.0),        # m/s -> drives wave size (Hs ~ 0.02-0.10 m)
        "steepness": (0.10, 0.30),
    },
    "river_dry": {
        "label": "River (dry season)",
        "current_speed": (0.5, 1.2),
        "wind_speed": (1.0, 2.5),        # Hs ~ 0.025-0.15 m
        "steepness": (0.10, 0.40),
    },
    "river_rain": {
        "label": "River (rain season)",
        "current_speed": (2.0, 3.0),
        "wind_speed": (1.5, 3.0),        # Hs ~ 0.05-0.22 m (rain gusts, report: 0.4-3.8 m/s at 30 cm)
        "steepness": (0.20, 0.50),
    },
}
ENV_CHOICES = [
    ("Lake (0.05-0.3 m/s)", "lake"),
    ("River, dry season (0.5-1.2 m/s)", "river_dry"),
    ("River, rain season (2.0-3.0 m/s)", "river_rain"),
    ("Mixed (random category each run)", "mixed"),
    ("Custom (use the ranges below)", "custom"),
]

# Custom-environment defaults (only used when Environment = custom)
CUSTOM_CURRENT_RANGE_DEFAULT = (0.0, 1.0)
WIND_SPEED_RANGE_DEFAULT = (1.0, 3.0)
STEEPNESS_RANGE_DEFAULT = (0.1, 0.5)
WAVE_ANGLE_RANGE_DEFAULT = (0.0, 360.0)
CURRENT_ANGLE_RANGE_DEFAULT = (0.0, 360.0)

CURRENT_RAMP_DEFAULT = 2.0        # s, linear ramp-up of the current at the start of each phase
DIVERGENCE_ABORT_M = 60.0         # abort a phase if |cross| or backwards progress exceeds this

MODE_PHASES = {
    "compare": [("no_pid", False), ("pid", False)],
    "pid_tuning": [("pid", False)],
    "xte_compare": [("pid_xte", False), ("pid_no_xte", True)],
    "full_real": [("no_pid", False), ("pid_xte", False), ("pid_no_xte", True)],
}
PLOT_MODES = {
    "compare": ("no_pid", "pid"),
    "pid_tuning": ("pid",),
    "xte_compare": ("pid_no_xte", "pid_xte"),
    "full_real": ("no_pid", "pid_no_xte", "pid_xte"),
}
PHASE_STYLE = {  # color, label, marker
    "no_pid": ("crimson", "No PID", "o"),
    "pid": ("royalblue", "PID", "^"),
    "pid_no_xte": ("darkorange", "PID, no XTE terms", "s"),
    "pid_xte": ("royalblue", "PID, with XTE terms", "^"),
}

CSV_FIELDS = [
    "run", "seed", "environment", "current_speed_mps", "current_angle_deg",
    "wave_wind_speed_mps", "wave_dir_deg", "steepness",
    "mode", "max_abs_cross_track_m", "final_abs_cross_track_m",
    "final_along_track_m", "arrived", "duration_s", "note",
]
TS_CSV_FIELDS = [
    "run", "mode", "t_s", "x_m", "y_m", "yaw_deg", "heading_error_deg",
    "cross_track_m", "cross_track_vel_mps", "cross_track_accel_mps2",
    "along_track_m", "left_thrust_N", "right_thrust_N",
]

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cross_along_track(x, y):
    px = x - START_POINT[0]
    py = y - START_POINT[1]
    along = px * LINE_DIR[0] + py * LINE_DIR[1]
    cross = -px * LINE_DIR[1] + py * LINE_DIR[0]
    return cross, along


def load_gz_bindings():
    candidates = [
        ("gz.transport", "gz.msgs"),
        ("gz.transport15", "gz.msgs12"),
        ("gz.transport14", "gz.msgs11"),
        ("gz.transport13", "gz.msgs10"),
        ("gz.transport12", "gz.msgs9"),
    ]
    for t_mod, m_mod in candidates:
        try:
            transport = importlib.import_module(t_mod)
            double_cls = importlib.import_module(f"{m_mod}.double_pb2").Double
            posev_cls = importlib.import_module(f"{m_mod}.pose_v_pb2").Pose_V
            vec3_cls = importlib.import_module(f"{m_mod}.vector3d_pb2").Vector3d
            return transport.Node, double_cls, posev_cls, vec3_cls
        except ImportError:
            continue
    return None, None, None, None


NodeCls, DoubleMsg, PoseVMsg, Vector3dMsg = load_gz_bindings()


def gz_cmd(args):
    return (["wsl", "--"] + args) if USE_WSL else args


def gz_service(service, reqtype, reptype, req, timeout_ms=2000):
    try:
        return subprocess.run(
            gz_cmd(["gz", "service", "-s", service,
                    "--reqtype", reqtype, "--reptype", reptype,
                    "--timeout", str(timeout_ms), "--req", req]),
            capture_output=True, text=True,
        )
    except FileNotFoundError as e:
        class _Result:
            returncode = -1
            stdout = ""
            stderr = str(e)
        return _Result()


def draw_environment(run_idx, run_seed, cfg):
    """Draws all environment parameters for one run from run_seed (so a
    batch seed reproduces the exact same environments)."""
    rng = random.Random(run_seed)
    key = cfg["environment"]
    if key == "mixed":
        key = rng.choice(list(ENV_PRESETS))
    if key == "custom":
        p = {"label": "Custom", "current_speed": cfg["custom_current_range"],
             "wind_speed": cfg["wind_speed_range"], "steepness": cfg["steepness_range"]}
    else:
        p = ENV_PRESETS[key]

    wind_speed = rng.uniform(*p["wind_speed"])
    wave_dir = rng.uniform(*cfg["wave_angle_range"])
    steepness = rng.uniform(*p["steepness"])
    cur_speed = rng.uniform(*p["current_speed"])

    lo, hi = cfg["current_angle_range"]
    if cfg["current_angle_mode"] == "sweep":
        n = cfg["num_runs"]
        divisor = n if (hi - lo) >= 360.0 else max(n - 1, 1)
        cur_angle = lo + (hi - lo) * (run_idx - 1) / divisor
    else:
        cur_angle = rng.uniform(lo, hi)

    return {
        "key": key, "label": p["label"],
        "current_speed": cur_speed, "current_angle": cur_angle,
        "wind_speed": wind_speed, "wave_dir": wave_dir, "steepness": steepness,
    }


def write_wave_model(env):
    if not WAVE_TEMPLATE_PATH.exists():
        print(f"Wave template not found at {WAVE_TEMPLATE_PATH} - skipping wave update.")
        return False
    if not WAVE_REAL_SDF_PATH.parent.exists():
        print(f"Wave model directory not found at {WAVE_REAL_SDF_PATH.parent} - skipping wave update.")
        return False

    backup_path = WAVE_REAL_SDF_PATH.with_suffix(WAVE_REAL_SDF_PATH.suffix + ".orig_bak")
    if WAVE_REAL_SDF_PATH.exists() and not backup_path.exists():
        shutil.copy2(WAVE_REAL_SDF_PATH, backup_path)

    text = WAVE_TEMPLATE_PATH.read_text()
    text = text.replace("__WIND_SPEED__", f"{env['wind_speed']:.3f}")
    text = text.replace("__WIND_ANGLE_DEG__", f"{env['wave_dir']:.2f}")
    text = text.replace("__STEEPNESS__", f"{env['steepness']:.3f}")
    if "__" in text:
        print("Warning: template still has an unfilled placeholder - check WAVE_TEMPLATE_PATH.")
    WAVE_REAL_SDF_PATH.write_text(text)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] waves: wind_speed={env['wind_speed']:.3f} m/s  "
          f"dir={env['wave_dir']:.1f} deg  steepness={env['steepness']:.3f}")
    return True


def patch_real_time_factor(new_rtf):
    if new_rtf is None or abs(new_rtf - 1.0) < 1e-9:
        return None
    if not WORLD_FILE_PATH.exists():
        print(f"World file not found at {WORLD_FILE_PATH} - cannot patch real_time_factor.")
        return None
    backup_path = WORLD_FILE_PATH.with_suffix(WORLD_FILE_PATH.suffix + ".rtf_bak")
    text = WORLD_FILE_PATH.read_text()
    if not backup_path.exists():
        backup_path.write_text(text)
    new_text = re.sub(
        r"<real_time_factor>[^<]*</real_time_factor>",
        f"<real_time_factor>{new_rtf}</real_time_factor>",
        text,
    )
    WORLD_FILE_PATH.write_text(new_text)
    print(f"Patched real_time_factor to {new_rtf} (backup at {backup_path})")
    return backup_path


def restore_real_time_factor(backup_path):
    if backup_path is not None and backup_path.exists():
        WORLD_FILE_PATH.write_text(backup_path.read_text())
        print("Restored original real_time_factor.")


_current_pub = None
_current_vec = (0.0, 0.0)


def init_current_publisher(node):
    global _current_pub, _current_vec
    _current_pub = node.advertise(CURRENT_TOPIC, Vector3dMsg)
    _current_vec = (0.0, 0.0)


def set_current_vector(speed, angle_deg):
    """Water flows TOWARD angle_deg (world frame, CCW from +X)."""
    global _current_vec
    a = math.radians(angle_deg)
    _current_vec = (speed * math.cos(a), speed * math.sin(a))


def _publish_vec(vx, vy):
    if _current_pub is None:
        return
    msg = Vector3dMsg()
    msg.x, msg.y, msg.z = vx, vy, 0.0
    _current_pub.publish(msg)


def publish_current(scale=1.0):
    _publish_vec(_current_vec[0] * scale, _current_vec[1] * scale)


def stop_current():
    """Zero the current between phases so the boat doesn't drift while the
    world is being reset."""
    for _ in range(3):
        _publish_vec(0.0, 0.0)
        time.sleep(TICK_S)


_active_gz_proc = None


def launch_world_background(headless=True):
    global _active_gz_proc
    if not WORLD_FILE_PATH.exists():
        print(f"World file not found at {WORLD_FILE_PATH} - not launching gz sim.")
        return None
    args = ["gz", "sim", "-r"]
    if headless:
        args.append("-s")
    args.append(str(WORLD_FILE_PATH))
    cmd = gz_cmd(args)
    print(f"  Launching: {' '.join(cmd)}")
    kwargs = {}
    if os.name == "posix":
        kwargs["preexec_fn"] = os.setsid
    try:
        proc = subprocess.Popen(cmd, **kwargs)
        _active_gz_proc = proc
        return proc
    except FileNotFoundError:
        print("Could not find 'gz' (or 'wsl'). Check PATH / USE_WSL setting.")
        return None


def terminate_gz_process(proc):
    global _active_gz_proc
    if proc is None:
        return
    try:
        if os.name == "posix":
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        else:
            proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        pass
    if _active_gz_proc is proc:
        _active_gz_proc = None


def _cleanup_on_exit():
    terminate_gz_process(_active_gz_proc)


atexit.register(_cleanup_on_exit)


def reset_world(world_name):
    """WorldControl reset: boat back at spawn, sim time 0, same process, so
    waves stay identical between phases."""
    req = "reset: {all: true}"
    result = gz_service(f"/world/{world_name}/control", "gz.msgs.WorldControl", "gz.msgs.Boolean", req)
    if getattr(result, "returncode", -1) != 0:
        print(f"  WARNING: world reset call failed: {getattr(result, 'stderr', '').strip()}")
    return result



class PoseState:
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None
        self.last_update = None
        self._logged_names = False

    def callback(self, msg):
        self.last_update = time.time()
        if not self._logged_names:
            names = sorted({p.name for p in msg.pose})
            print(f"  [pose] entities seen on topic: {names}")
            if MODEL_NAME not in names:
                print(f"  [pose] WARNING: '{MODEL_NAME}' not in that list - check MODEL_NAME.")
            self._logged_names = True
        for pose in msg.pose:
            if pose.name == MODEL_NAME:
                q = pose.orientation
                self.yaw = wrap_pi(yaw_from_quat(q.x, q.y, q.z, q.w) - BODY_YAW_OFFSET)
                self.x = pose.position.x
                self.y = pose.position.y
                return


def wait_for_pose(state, timeout=15.0, want_reset_near_start=False):
    start = time.time()
    while True:
        if state.x is not None:
            if not want_reset_near_start:
                return True
            _, along = cross_along_track(state.x, state.y)
            if abs(along) < 0.5:
                return True
        if time.time() - start > timeout:
            return state.x is not None
        time.sleep(0.1)


def reset_and_wait(state):
    reset_world(WORLD_NAME)
    time.sleep(1.5)
    wait_for_pose(state, timeout=5.0, want_reset_near_start=True)


def verify_current_response(state, test_speed=1.0, duration=4.0):
    print(f"  [check] applying {test_speed} m/s current along +X with zero thrust for {duration:.0f}s...")
    set_current_vector(test_speed, 0.0)
    x0 = state.x
    t0 = time.time()
    while time.time() - t0 < duration:
        publish_current(1.0)
        time.sleep(TICK_S)
    elapsed = time.time() - t0
    drift_v = (state.x - x0) / elapsed if x0 is not None and state.x is not None else float("nan")
    stop_current()
    if drift_v == drift_v and drift_v > 0.1:
        print(f"  [check] OK: boat drifted at ~{drift_v:.2f} m/s -> /ocean_current is being applied.")
    else:
        print(f"  [check] WARNING: boat drift along +X was only {drift_v:.3f} m/s. "
              f"The plugin on the hull probably ignores {CURRENT_TOPIC}; current results will NOT be valid.")
    reset_and_wait(state)
    return drift_v



def run_phase(state, pub_left, pub_right, mode, cfg, run_idx):
    integral = 0.0
    prev_error = 0.0
    max_abs_cross = 0.0
    last_cross = 0.0
    along = 0.0
    start_time = time.time()
    last_tick = start_time
    arrived = False
    diverged = False
    timed_out = False
    target_yaw = LINE_HEADING
    records = []

    prev_cross = None
    filt_cross_vel = 0.0
    prev_filt_cross_vel = 0.0
    filt_cross_accel = 0.0
    drift_tau = cfg.get("drift_tau", DRIFT_FILTER_TAU_DEFAULT)
    ramp = cfg.get("current_ramp_s", CURRENT_RAMP_DEFAULT)

    while True:
        now = time.time()
        dt = now - last_tick if now > last_tick else TICK_S
        last_tick = now
        elapsed = now - start_time

        if elapsed > cfg["timeout_s"]:
            timed_out = True
            break

        if state.x is None:
            time.sleep(TICK_S)
            continue

        cross, along = cross_along_track(state.x, state.y)
        max_abs_cross = max(max_abs_cross, abs(cross))
        last_cross = cross

        if abs(cross) > DIVERGENCE_ABORT_M or along < -DIVERGENCE_ABORT_M:
            diverged = True
            break

        if along >= LINE_LENGTH - GOAL_RADIUS:
            arrived = True
            break

        heading_error = wrap_pi(target_yaw - state.yaw) if state.yaw is not None else None

        raw_cross_vel = 0.0 if prev_cross is None else (cross - prev_cross) / dt if dt > 0 else 0.0
        prev_cross = cross
        alpha_v = dt / (drift_tau + dt) if (drift_tau + dt) > 0 else 1.0
        prev_filt_cross_vel = filt_cross_vel
        filt_cross_vel += alpha_v * (raw_cross_vel - filt_cross_vel)

        raw_cross_accel = (filt_cross_vel - prev_filt_cross_vel) / dt if dt > 0 else 0.0
        filt_cross_accel += alpha_v * (raw_cross_accel - filt_cross_accel)

        if mode == "no_pid":
            left = right = clamp(cfg["base_thrust"], 0.0, MAX_THRUST)
        else:
            integral += (heading_error or 0.0) * dt
            derivative = ((heading_error or 0.0) - prev_error) / dt if dt > 0 else 0.0
            prev_error = heading_error or 0.0
            correction = (cfg["kp"] * (heading_error or 0.0) + cfg["ki"] * integral +
                          cfg["kd"] * derivative - cfg["kxte"] * cross -
                          cfg.get("kxte_dot", 0.0) * filt_cross_vel -
                          cfg.get("kxte_ddot", 0.0) * filt_cross_accel)
            left = clamp(cfg["base_thrust"] - correction, 0.0, MAX_THRUST)
            right = clamp(cfg["base_thrust"] + correction, 0.0, MAX_THRUST)

        records.append({
            "t": elapsed,
            "x": state.x,
            "y": state.y,
            "yaw_deg": math.degrees(state.yaw) if state.yaw is not None else float("nan"),
            "heading_error_deg": math.degrees(heading_error) if heading_error is not None else float("nan"),
            "cross_track_m": cross,
            "cross_track_vel_mps": filt_cross_vel,
            "cross_track_accel_mps2": filt_cross_accel,
            "along_track_m": along,
            "left_thrust_N": left,
            "right_thrust_N": right,
        })

        publish_current(min(1.0, elapsed / ramp) if ramp > 0 else 1.0)
        lmsg, rmsg = DoubleMsg(), DoubleMsg()
        lmsg.data, rmsg.data = left, right
        pub_left.publish(lmsg)
        pub_right.publish(rmsg)

        print(f"\r  run {run_idx} [{mode:10s}] t={elapsed:6.1f}s along={along:6.2f}/{LINE_LENGTH:.1f}m "
              f"cross={cross:+6.2f}m (max {max_abs_cross:.2f}m)   ", end="", flush=True)

        time.sleep(TICK_S)

    lmsg, rmsg = DoubleMsg(), DoubleMsg()
    lmsg.data = rmsg.data = 0.0
    pub_left.publish(lmsg)
    pub_right.publish(rmsg)
    stop_current()
    print()

    note = "diverged" if diverged else ("timeout" if timed_out else "")
    metrics = {
        "max_abs_cross_track_m": max_abs_cross,
        "final_abs_cross_track_m": abs(last_cross),
        "final_along_track_m": along,
        "arrived": arrived,
        "duration_s": time.time() - start_time,
        "note": note,
    }
    return metrics, records


def _fmt(v, digits=4):
    return f"{v:.{digits}f}" if isinstance(v, float) and v == v else ("" if v is None else v)


def append_csv_row(csv_path, run_idx, seed, env, mode, metrics=None, note=""):
    write_header = not os.path.exists(csv_path)
    metrics = metrics or {}
    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow({
            "run": run_idx,
            "seed": seed,
            "environment": env["key"],
            "current_speed_mps": _fmt(env["current_speed"], 3),
            "current_angle_deg": _fmt(env["current_angle"], 1),
            "wave_wind_speed_mps": _fmt(env["wind_speed"], 3),
            "wave_dir_deg": _fmt(env["wave_dir"], 2),
            "steepness": _fmt(env["steepness"], 3),
            "mode": mode,
            "max_abs_cross_track_m": _fmt(metrics.get("max_abs_cross_track_m"), 4),
            "final_abs_cross_track_m": _fmt(metrics.get("final_abs_cross_track_m"), 4),
            "final_along_track_m": _fmt(metrics.get("final_along_track_m"), 4),
            "arrived": metrics.get("arrived", ""),
            "duration_s": _fmt(metrics.get("duration_s"), 2),
            "note": note or metrics.get("note", ""),
        })


def append_timeseries_rows(csv_path, run_idx, mode, records):
    if not records:
        return
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=TS_CSV_FIELDS)
        if write_header:
            writer.writeheader()
        for rec in records:
            writer.writerow({
                "run": run_idx,
                "mode": mode,
                "t_s": f"{rec['t']:.3f}",
                "x_m": f"{rec['x']:.4f}",
                "y_m": f"{rec['y']:.4f}",
                "yaw_deg": f"{rec['yaw_deg']:.3f}",
                "heading_error_deg": f"{rec['heading_error_deg']:.3f}",
                "cross_track_m": f"{rec['cross_track_m']:.4f}",
                "cross_track_vel_mps": f"{rec['cross_track_vel_mps']:.4f}",
                "cross_track_accel_mps2": f"{rec['cross_track_accel_mps2']:.4f}",
                "along_track_m": f"{rec['along_track_m']:.4f}",
                "left_thrust_N": f"{rec['left_thrust_N']:.3f}",
                "right_thrust_N": f"{rec['right_thrust_N']:.3f}",
            })


def run_single(run_idx, run_seed, cfg, csv_path, ts_csv_path):
    mode = cfg["mode"]
    phases = MODE_PHASES[mode]
    env = draw_environment(run_idx, run_seed, cfg)

    print(f"\n=== [{mode}] Run {run_idx}/{cfg['num_runs']}  (seed={run_seed}) ===")
    print(f"  Environment: {env['label']}  current={env['current_speed']:.2f} m/s "
          f"flowing toward {env['current_angle']:.0f} deg")
    write_wave_model(env)

    def log_failure(note):
        for name, _ in phases:
            append_csv_row(csv_path, run_idx, run_seed, env, name, note=note)

    proc = launch_world_background(headless=cfg["headless"])
    if proc is None:
        print(f"  Skipping run {run_idx}: could not launch gz sim.")
        log_failure("launch_failed")
        return

    try:
        time.sleep(cfg["gz_boot_wait_s"])

        node = NodeCls()
        pub_left = node.advertise(LEFT_TOPIC, DoubleMsg)
        pub_right = node.advertise(RIGHT_TOPIC, DoubleMsg)
        init_current_publisher(node)
        state = PoseState()
        if not node.subscribe(PoseVMsg, POSE_TOPIC, state.callback):
            print(f"  WARNING: failed to subscribe to {POSE_TOPIC}")

        if not wait_for_pose(state, timeout=15.0):
            print(f"  Run {run_idx}: never received pose data on {POSE_TOPIC} - aborting this run.")
            log_failure("no_pose_data")
            return

        if run_idx == 1 and cfg.get("verify_current", True):
            verify_current_response(state)

        for i, (name, disable_xte) in enumerate(phases):
            if i > 0:
                print(f"  Resetting world '{WORLD_NAME}' before {name} phase...")
                reset_and_wait(state)

            phase_cfg = dict(cfg)
            if disable_xte:
                phase_cfg["kxte"] = phase_cfg["kxte_dot"] = phase_cfg["kxte_ddot"] = 0.0

            set_current_vector(env["current_speed"], env["current_angle"])
            metrics, records = run_phase(state, pub_left, pub_right, name, phase_cfg, run_idx)
            append_csv_row(csv_path, run_idx, run_seed, env, name, metrics)
            append_timeseries_rows(ts_csv_path, run_idx, name, records)
            print(f"  {name:10s}: max|cross|={metrics['max_abs_cross_track_m']:.2f} m  "
                  f"final|cross|={metrics['final_abs_cross_track_m']:.2f} m  "
                  f"arrived={metrics['arrived']}  t={metrics['duration_s']:.1f}s  {metrics['note']}")
    finally:
        terminate_gz_process(proc)
        time.sleep(1.0)


CONFIG_SAVE_PATH = Path(__file__).resolve().parent / "batch_test_runner_last_config.json"

_PERSISTED_KEYS = [
    "mode", "environment", "ca_mode", "runs", "seed", "timeout", "boot", "base",
    "kp", "ki", "kd", "kxte", "kxte_dot", "kxte_ddot", "drift_tau",
    "cur_min", "cur_max", "ca_min", "ca_max", "ramp",
    "ws_min", "ws_max", "wa_min", "wa_max", "st_min", "st_max", "rtf",
    "csv", "ts_csv", "plot", "headless", "verify",
]


def load_last_config():
    if not CONFIG_SAVE_PATH.exists():
        return {}
    try:
        with open(CONFIG_SAVE_PATH, "r") as f:
            data = json.load(f)
        return {k: v for k, v in data.items() if k in _PERSISTED_KEYS}
    except Exception as e:
        print(f"Could not read saved config at {CONFIG_SAVE_PATH} ({e}) - using defaults.")
        return {}


def save_last_config(values):
    try:
        CONFIG_SAVE_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(CONFIG_SAVE_PATH, "w") as f:
            json.dump(values, f, indent=2)
        print(f"Saved config settings to {CONFIG_SAVE_PATH}")
    except Exception as e:
        print(f"WARNING: could not save config settings to {CONFIG_SAVE_PATH}: {e}")


def show_config_dialog():
    result = {}
    started = {"ok": False}
    saved = load_last_config()

    def sv(key, default):
        return saved[key] if key in saved else default

    root = tk.Tk()
    root.title("Autonomous Boat Test — Batch Config")

    fields = {}

    # Three side-by-side columns so the window stays short.
    col_a = ttk.Frame(root)
    col_b = ttk.Frame(root)
    col_c = ttk.Frame(root)
    col_a.grid(row=0, column=0, sticky="nw", padx=8, pady=6)
    col_b.grid(row=0, column=1, sticky="nw", padx=8, pady=6)
    col_c.grid(row=0, column=2, sticky="nw", padx=8, pady=6)

    def add_group(parent, title):
        g = ttk.LabelFrame(parent, text=title)
        g.pack(fill="x", pady=(0, 8))
        g.columnconfigure(1, weight=1)
        return g

    def add_radio_group(parent, title, var, options):
        g = ttk.LabelFrame(parent, text=title)
        g.pack(fill="x", pady=(0, 8))
        for text, value in options:
            ttk.Radiobutton(g, text=text, variable=var, value=value).pack(anchor="w", padx=6)

    def add_row(group, label, key, default):
        r = group.grid_size()[1]
        ttk.Label(group, text=label).grid(row=r, column=0, sticky="w", padx=6, pady=2)
        v = tk.StringVar(value=str(sv(key, default)))
        ttk.Entry(group, textvariable=v, width=12).grid(row=r, column=1, sticky="e", padx=6, pady=2)
        fields[key] = v

    mode_var = tk.StringVar(value=sv("mode", "compare"))
    add_radio_group(col_a, "Mode", mode_var, [
        ("Compare (No PID vs PID)", "compare"),
        ("PID tuning (PID only + error plots)", "pid_tuning"),
        ("XTE compare (PID with vs without XTE)", "xte_compare"),
        ("Full real (No PID / PID+XTE / PID no XTE)", "full_real"),
    ])

    env_var = tk.StringVar(value=sv("environment", "river_dry"))
    add_radio_group(col_a, "Environment", env_var, ENV_CHOICES)

    ca_mode_var = tk.StringVar(value=sv("ca_mode", "random"))
    add_radio_group(col_a, "Current direction", ca_mode_var, [
        ("Random each run (within angle range)", "random"),
        ("Sweep (evenly spaced around circle)", "sweep"),
    ])

    g = add_group(col_a, "Batch")
    add_row(g, "Number of test runs", "runs", 8)
    add_row(g, "Batch seed (blank = random)", "seed", "")
    add_row(g, "Per-phase timeout (s)", "timeout", 90)
    add_row(g, "gz sim boot wait (s)", "boot", 3.0)
    add_row(g, "Real-time factor (1.0 = no change)", "rtf", 1.0)

    g = add_group(col_b, "Controller")
    add_row(g, "Base thrust (N)", "base", BASE_THRUST_DEFAULT)
    add_row(g, "PID Kp", "kp", KP_DEFAULT)
    add_row(g, "PID Ki", "ki", KI_DEFAULT)
    add_row(g, "PID Kd", "kd", KD_DEFAULT)
    add_row(g, "Kxte (cross-track position)", "kxte", KXTE_DEFAULT)
    add_row(g, "Kxte_dot (drift rate)", "kxte_dot", KXTE_DOT_DEFAULT)
    add_row(g, "Kxte_ddot (drift accel)", "kxte_ddot", KXTE_DDOT_DEFAULT)
    add_row(g, "Drift filter time const (s)", "drift_tau", DRIFT_FILTER_TAU_DEFAULT)

    g = add_group(col_b, "Current & wave direction (all environments)")
    add_row(g, "Current direction min (deg, 0=+X)", "ca_min", CURRENT_ANGLE_RANGE_DEFAULT[0])
    add_row(g, "Current direction max (deg)", "ca_max", CURRENT_ANGLE_RANGE_DEFAULT[1])
    add_row(g, "Current ramp-up time (s)", "ramp", CURRENT_RAMP_DEFAULT)
    add_row(g, "Wave direction min (deg)", "wa_min", WAVE_ANGLE_RANGE_DEFAULT[0])
    add_row(g, "Wave direction max (deg)", "wa_max", WAVE_ANGLE_RANGE_DEFAULT[1])

    g = add_group(col_c, "Custom environment (only if 'Custom')")
    add_row(g, "Current speed min (m/s)", "cur_min", CUSTOM_CURRENT_RANGE_DEFAULT[0])
    add_row(g, "Current speed max (m/s)", "cur_max", CUSTOM_CURRENT_RANGE_DEFAULT[1])
    add_row(g, "Wave wind speed min (m/s)", "ws_min", WIND_SPEED_RANGE_DEFAULT[0])
    add_row(g, "Wave wind speed max (m/s)", "ws_max", WIND_SPEED_RANGE_DEFAULT[1])
    add_row(g, "Steepness min", "st_min", STEEPNESS_RANGE_DEFAULT[0])
    add_row(g, "Steepness max", "st_max", STEEPNESS_RANGE_DEFAULT[1])

    g = add_group(col_c, "Output files")
    add_row(g, "Summary CSV", "csv", "batch_results.csv")
    add_row(g, "Time-series CSV", "ts_csv", "batch_timeseries.csv")
    add_row(g, "Plot base name", "plot", "batch_divergence_plot.png")

    g = add_group(col_c, "Options")
    headless_var = tk.BooleanVar(value=bool(sv("headless", True)))
    ttk.Checkbutton(g, text="Run gz sim headless (much faster)",
                    variable=headless_var).pack(anchor="w", padx=6, pady=2)
    verify_var = tk.BooleanVar(value=bool(sv("verify", True)))
    ttk.Checkbutton(g, text="Verify /ocean_current moves the boat (run 1)",
                    variable=verify_var).pack(anchor="w", padx=6, pady=2)

    ttk.Label(root, foreground="#555", wraplength=1000, justify="left",
              text="Presets (lake / river dry / river rain / mixed) set current speed, wave wind and "
                   "steepness themselves; the Custom fields are only used for 'Custom'. "
                   "Current and wave direction ranges apply to every environment."
              ).grid(row=1, column=0, columnspan=3, sticky="w", padx=8)

    def on_start():
        try:
            result["mode"] = mode_var.get()
            result["environment"] = env_var.get()
            result["current_angle_mode"] = ca_mode_var.get()
            result["num_runs"] = int(fields["runs"].get())
            if result["num_runs"] < 1:
                raise ValueError
            seed_str = fields["seed"].get().strip()
            result["seed"] = int(seed_str) if seed_str else None
            result["timeout_s"] = float(fields["timeout"].get())
            result["gz_boot_wait_s"] = float(fields["boot"].get())
            result["base_thrust"] = float(fields["base"].get())
            result["kp"] = float(fields["kp"].get())
            result["ki"] = float(fields["ki"].get())
            result["kd"] = float(fields["kd"].get())
            result["kxte"] = float(fields["kxte"].get())
            result["kxte_dot"] = float(fields["kxte_dot"].get())
            result["kxte_ddot"] = float(fields["kxte_ddot"].get())
            result["drift_tau"] = float(fields["drift_tau"].get())
            result["current_angle_range"] = (float(fields["ca_min"].get()), float(fields["ca_max"].get()))
            result["current_ramp_s"] = float(fields["ramp"].get())
            result["wave_angle_range"] = (float(fields["wa_min"].get()), float(fields["wa_max"].get()))
            result["custom_current_range"] = (float(fields["cur_min"].get()), float(fields["cur_max"].get()))
            result["wind_speed_range"] = (float(fields["ws_min"].get()), float(fields["ws_max"].get()))
            result["steepness_range"] = (float(fields["st_min"].get()), float(fields["st_max"].get()))
            result["real_time_factor"] = float(fields["rtf"].get())
            result["output_csv"] = fields["csv"].get().strip() or "batch_results.csv"
            result["output_timeseries_csv"] = fields["ts_csv"].get().strip() or "batch_timeseries.csv"
            result["output_plot"] = fields["plot"].get().strip() or "batch_divergence_plot.png"
            result["headless"] = bool(headless_var.get())
            result["verify_current"] = bool(verify_var.get())
        except ValueError:
            messagebox.showerror("Bad input", "Please check the values (numbers only; runs >= 1).")
            return

        to_save = {key: v.get() for key, v in fields.items()}
        to_save["mode"] = mode_var.get()
        to_save["environment"] = env_var.get()
        to_save["ca_mode"] = ca_mode_var.get()
        to_save["headless"] = bool(headless_var.get())
        to_save["verify"] = bool(verify_var.get())
        save_last_config(to_save)

        started["ok"] = True
        root.destroy()

    btn_frame = ttk.Frame(root)
    btn_frame.grid(row=2, column=0, columnspan=3, pady=8)
    ttk.Button(btn_frame, text="Start batch", command=on_start).pack(side="left", padx=6)
    ttk.Button(btn_frame, text="Cancel", command=root.destroy).pack(side="left", padx=6)

    root.mainloop()
    return result if started["ok"] else None


def print_summary_table(csv_path):
    groups = {}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if not row.get("max_abs_cross_track_m"):
                continue
            g = groups.setdefault((row["environment"], row["mode"]),
                                  {"n": 0, "arrived": 0, "max": [], "final": []})
            g["n"] += 1
            g["arrived"] += 1 if row["arrived"] == "True" else 0
            g["max"].append(float(row["max_abs_cross_track_m"]))
            g["final"].append(float(row["final_abs_cross_track_m"]))
    if not groups:
        return
    print("\nSummary by environment / mode:")
    print(f"  {'environment':12s} {'mode':11s} {'runs':>4s} {'arrived':>8s} {'mean max|xte|':>14s} {'mean final|xte|':>16s}")
    for (env, mode), g in sorted(groups.items()):
        print(f"  {env:12s} {mode:11s} {g['n']:4d} {g['arrived']:4d}/{g['n']:<3d} "
              f"{sum(g['max']) / g['n']:14.2f} {sum(g['final']) / g['n']:16.2f}")


def _plot_path_with_suffix(out_path, suffix):
    p = Path(out_path)
    return str(p.with_name(p.stem + suffix + p.suffix))


def _load_summary(csv_path, modes, value_col="max_abs_cross_track_m"):
    result = {m: ([], []) for m in modes}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if not row.get(value_col):
                continue
            mode = row["mode"]
            if mode not in result:
                continue
            result[mode][0].append(int(row["run"]))
            result[mode][1].append(abs(float(row[value_col])))
    return result


def _load_timeseries(ts_csv_path, modes):
    data = {m: {} for m in modes}
    if not os.path.exists(ts_csv_path):
        return data
    with open(ts_csv_path, newline="") as f:
        for row in csv.DictReader(f):
            mode = row["mode"]
            if mode not in data:
                continue
            run = int(row["run"])
            bucket = data[mode].setdefault(run, {"t": [], "cross": [], "yaw": [], "along": []})
            bucket["t"].append(float(row["t_s"]))
            bucket["cross"].append(abs(float(row["cross_track_m"])))
            bucket["yaw"].append(float(row["yaw_deg"]))
            bucket["along"].append(float(row["along_track_m"]))
    return data


def _load_timeseries_pid_tuning(ts_csv_path):
    data = {}
    if not os.path.exists(ts_csv_path):
        return data
    with open(ts_csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["mode"] != "pid":
                continue
            run = int(row["run"])
            b = data.setdefault(run, {
                "t": [], "heading_error": [], "cross_track": [],
                "cross_track_vel": [], "cross_track_accel": [],
                "left_thrust": [], "right_thrust": [],
            })
            b["t"].append(float(row["t_s"]))
            b["heading_error"].append(float(row["heading_error_deg"]))
            b["cross_track"].append(float(row["cross_track_m"]))
            b["cross_track_vel"].append(float(row.get("cross_track_vel_mps", 0.0) or 0.0))
            b["cross_track_accel"].append(float(row.get("cross_track_accel_mps2", 0.0) or 0.0))
            b["left_thrust"].append(float(row["left_thrust_N"]))
            b["right_thrust"].append(float(row["right_thrust_N"]))
    return data


def _mean_std_over_time(per_run_dict, key, grid):
    import numpy as np
    if not per_run_dict:
        return None, None
    stacked = []
    for series in per_run_dict.values():
        t = np.asarray(series["t"])
        v = np.asarray(series[key])
        if len(t) < 2:
            continue
        stacked.append(np.interp(grid, t, v, left=np.nan, right=np.nan))
    if not stacked:
        return None, None
    stacked = np.vstack(stacked)
    with np.errstate(all="ignore"):
        return np.nanmean(stacked, axis=0), np.nanstd(stacked, axis=0)


def _plot_mean_std_over_time(ts_data, key, ylabel, title, out_path, modes):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    max_t = 0.0
    for mode in modes:
        for series in ts_data[mode].values():
            if series["t"]:
                max_t = max(max_t, series["t"][-1])
    if max_t <= 0:
        return False
    grid = np.linspace(0.0, max_t, 200)

    plt.figure(figsize=(9, 6))
    plotted_any = False
    for mode in modes:
        mean, std = _mean_std_over_time(ts_data[mode], key, grid)
        if mean is None:
            continue
        plotted_any = True
        color, label, _ = PHASE_STYLE[mode]
        plt.plot(grid, mean, color=color, label=label, linewidth=2)
        plt.fill_between(grid, mean - std, mean + std, color=color, alpha=0.2)
    if not plotted_any:
        plt.close()
        return False

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title, fontsize=10)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved plot to {out_path}")
    return True


def _plot_divergence_scatter(csv_path, out_path, title, value_col, ylabel, modes):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    summary = _load_summary(csv_path, modes, value_col)
    fig, ax = plt.subplots(figsize=(9, 6))
    plotted_any = False
    for mode in modes:
        runs, vals = summary[mode]
        if not runs:
            continue
        plotted_any = True
        color, label, marker = PHASE_STYLE[mode]
        ax.scatter(runs, vals, color=color, label=label, marker=marker, s=60)
    ax.set_xlabel("Run number")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if plotted_any:
        ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {out_path}")
    return plotted_any


def _gains_str(cfg):
    return (f"Kp={cfg['kp']:g}  Ki={cfg['ki']:g}  Kd={cfg['kd']:g}  Kxte={cfg['kxte']:g}  "
            f"Kxte_dot={cfg.get('kxte_dot', 0.0):g}  Kxte_ddot={cfg.get('kxte_ddot', 0.0):g}")


def make_comparison_plots(csv_path, ts_csv_path, out_path, cfg, modes):
    names = " vs ".join(PHASE_STYLE[m][1] for m in modes)
    g = f"[{_gains_str(cfg)}]"

    _plot_divergence_scatter(csv_path, out_path, f"Path divergence (max): {names}\n{g}",
                             "max_abs_cross_track_m", "Max |cross-track divergence| (m)", modes)
    _plot_divergence_scatter(csv_path, _plot_path_with_suffix(out_path, "_final_divergence"),
                             f"Path divergence (final): {names}\n{g}",
                             "final_abs_cross_track_m", "Final |cross-track divergence| (m)", modes)

    ts = _load_timeseries(ts_csv_path, modes)
    _plot_mean_std_over_time(ts, "cross", "|Cross-track error| (m)",
                             f"Mean |cross-track error| over time — {names}\n{g}",
                             _plot_path_with_suffix(out_path, "_cross_track_error_time"), modes)
    _plot_mean_std_over_time(ts, "yaw", "Heading / yaw (deg)",
                             f"Mean heading over time — {names}\n{g}",
                             _plot_path_with_suffix(out_path, "_heading_time"), modes)
    _plot_mean_std_over_time(ts, "along", "Along-track progress (m)",
                             f"Mean along-track progress over time — {names}\n{g}",
                             _plot_path_with_suffix(out_path, "_progress_time"), modes)

    if {"no_pid", "pid_xte", "pid_no_xte"} <= set(modes):
        _plot_mean_std_over_time(ts, "cross", "|Cross-track error| (m)",
                                 f"Mean |cross-track error| — PID (no XTE) vs PID (XTE) only\n{g}",
                                 _plot_path_with_suffix(out_path, "_xte_only_cross_track_error_time"),
                                 ("pid_no_xte", "pid_xte"))


def make_current_direction_plot(csv_path, out_path, modes, value_col="max_abs_cross_track_m"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pts = {m: ([], []) for m in modes}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            m = row["mode"]
            if m not in pts or not row.get(value_col) or not row.get("current_angle_deg"):
                continue
            pts[m][0].append(math.radians(float(row["current_angle_deg"])))
            pts[m][1].append(abs(float(row[value_col])))
    if not any(pts[m][0] for m in modes):
        return False

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    for m in modes:
        th, r = pts[m]
        if not th:
            continue
        color, label, marker = PHASE_STYLE[m]
        ax.scatter(th, r, color=color, label=label, marker=marker, s=60, alpha=0.85)
    ax.set_title("Max |cross-track| (m) vs current direction\n"
                 "0° = flows along +X (following), 180° = head-on, 90°/270° = beam", fontsize=10, pad=20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.25, 1.1))
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {out_path}")
    return True


def make_pid_tuning_plot(ts_csv_path, out_path, cfg):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ts_data = _load_timeseries_pid_tuning(ts_csv_path)
    if not ts_data:
        print("No PID time-series data found - skipping PID tuning plot.")
        return False

    fig, (ax_head, ax_cross, ax_drift, ax_thrust) = plt.subplots(4, 1, figsize=(10, 14), sharex=True)
    cmap = plt.get_cmap("tab10")
    max_labeled = 10

    for i, (run_idx, s) in enumerate(sorted(ts_data.items())):
        color = cmap(i % 10)
        label = f"run {run_idx}" if i < max_labeled else None
        ax_head.plot(s["t"], s["heading_error"], color=color, linewidth=1.4, label=label)
        ax_cross.plot(s["t"], s["cross_track"], color=color, linewidth=1.4, label=label)
        ax_drift.plot(s["t"], s["cross_track_vel"], color=color, linewidth=1.3,
                      label=(f"{label} vel" if label else None))
        ax_drift.plot(s["t"], s["cross_track_accel"], color=color, linewidth=1.0, linestyle="--",
                      alpha=0.7, label=(f"{label} accel" if label else None))
        ax_thrust.plot(s["t"], s["left_thrust"], color=color, linewidth=1.2,
                       label=(f"{label} L" if label else None))
        ax_thrust.plot(s["t"], s["right_thrust"], color=color, linewidth=1.2, linestyle="--",
                       label=(f"{label} R" if label else None))

    ax_head.axhline(0.0, color="black", linewidth=0.8, linestyle=":")
    ax_head.set_ylabel("Heading error (deg)")
    ax_head.set_title(f"PID tuning diagnostics\n{_gains_str(cfg)}  (base thrust={cfg['base_thrust']:g} N)",
                      fontsize=10)
    ax_head.grid(alpha=0.3)
    ax_head.legend(fontsize=8, ncol=2, loc="upper right")

    ax_cross.axhline(0.0, color="black", linewidth=0.8, linestyle=":")
    ax_cross.set_ylabel("Cross-track error (m)")
    ax_cross.grid(alpha=0.3)

    ax_drift.axhline(0.0, color="black", linewidth=0.8, linestyle=":")
    ax_drift.set_ylabel("Cross-track drift\n(solid=vel m/s, dashed=accel m/s²)")
    ax_drift.grid(alpha=0.3)
    ax_drift.legend(fontsize=7, ncol=2, loc="upper right")

    ax_thrust.axhline(cfg["base_thrust"], color="black", linewidth=0.8, linestyle=":")
    ax_thrust.set_ylabel("Thrust (N)\n(solid=left, dashed=right)")
    ax_thrust.set_xlabel("Time (s)")
    ax_thrust.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved PID tuning diagnostics plot to {out_path}")
    return True

def main():
    if NodeCls is None:
        print(
            "gz-transport Python bindings not found. Install e.g.:\n"
            "  sudo apt install python3-gz-transport14\n"
            "(or python3-gz-transport12 / 13 / 15, matching your gz version)"
        )
        return

    cfg = show_config_dialog()
    if cfg is None:
        print("Cancelled.")
        return

    print("Batch config:")
    for k, v in cfg.items():
        print(f"  {k}: {v}")

    for path_key in ("output_csv", "output_timeseries_csv"):
        p = Path(cfg[path_key])
        if p.exists():
            print(f"Removing previous output file: {p}")
            p.unlink()

    seed_master = random.Random(cfg["seed"]) if cfg["seed"] is not None else random.SystemRandom()
    mode = cfg["mode"]

    rtf_backup = patch_real_time_factor(cfg["real_time_factor"])
    try:
        for run_idx in range(1, cfg["num_runs"] + 1):
            run_seed = seed_master.randrange(2 ** 31)
            try:
                run_single(run_idx, run_seed, cfg, cfg["output_csv"], cfg["output_timeseries_csv"])
            except Exception as e:
                print(f"  ERROR during run {run_idx}: {e}")
                terminate_gz_process(_active_gz_proc)
    finally:
        restore_real_time_factor(rtf_backup)

    print(f"\nAll runs complete. Summary results in {cfg['output_csv']}")
    print(f"Time-series results in {cfg['output_timeseries_csv']}")
    if not os.path.exists(cfg["output_csv"]):
        return
    print_summary_table(cfg["output_csv"])

    modes = PLOT_MODES[mode]
    try:
        if mode == "pid_tuning":
            make_pid_tuning_plot(cfg["output_timeseries_csv"],
                                 _plot_path_with_suffix(cfg["output_plot"], "_pid_tuning"), cfg)
        else:
            base = {"compare": cfg["output_plot"],
                    "xte_compare": _plot_path_with_suffix(cfg["output_plot"], "_xte_compare"),
                    "full_real": _plot_path_with_suffix(cfg["output_plot"], "_full_real")}[mode]
            make_comparison_plots(cfg["output_csv"], cfg["output_timeseries_csv"], base, cfg, modes)
        make_current_direction_plot(cfg["output_csv"],
                                    _plot_path_with_suffix(cfg["output_plot"], "_current_direction"), modes)
    except ImportError:
        print("matplotlib/numpy not installed - install with:\n  pip install matplotlib numpy --break-system-packages")


if __name__ == "__main__":
    main()
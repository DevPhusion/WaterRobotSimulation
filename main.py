#!/usr/bin/env python3
"""
Autonomous batch tester for the gz-sim dual-thruster boat: No-PID vs PID
divergence comparison, headless, over N randomized-wave runs.

WHAT THIS DOES
    1. Pops up a small config window (number of runs, seed, timeouts, PID
       gains, wave ranges, headless on/off, output paths).
    2. For each run:
         a. Randomizes wind speed/direction + wave steepness and writes them
            into the waves model .sdf (same mechanism as main.py).
         b. Launches `gz sim` headless (server only, no rendering -> fast).
         c. Drives the boat with NO controller (constant thrust) along the
            fixed START_POINT -> END_POINT line, tracking cross-track error.
         d. Calls the world's WorldControl "reset" service (puts the boat
            back at spawn, resets sim time to 0) - SAME wave field, so the
            two phases are a fair paired comparison.
         e. Re-runs the same line with the heading PID controller active.
         f. Appends both phases' results (max |cross-track| divergence,
            final progress, arrival, duration) to a summary CSV, and every
            control-loop tick (time, position, heading, cross/along-track,
            thrust) to a separate time-series CSV.
         g. Kills this run's gz sim process before starting the next run,
            since the wave field is only re-read at world load time.
    3. After all runs, produces several plots:
         - Scatter of |max cross-track divergence| per run: No PID vs PID
           (integer-only run-number x-axis).
         - Mean +/- std of |cross-track error| over time, No PID vs PID.
         - Mean +/- std of heading (yaw, deg) over time, No PID vs PID.
         - Mean +/- std of along-track progress over time, No PID vs PID.

    NOTE ON OUTPUT FILES: both the summary CSV and the time-series CSV are
    reset (deleted, if present) at the START of each batch run, so that
    results from a previous batch never get mixed in with the current one
    (this used to cause duplicate/overlapping run numbers and incorrect
    plots when re-running the script against an old output path).

REQUIREMENTS
    - `gz` CLI on PATH (same as main.py).
    - gz-transport / gz-msgs python bindings (same as main.py):
          sudo apt install python3-gz-transport14   # or 12/13/15
    - matplotlib + numpy:
          pip install matplotlib numpy --break-system-packages
    - The constants below (WORLD_NAME, MODEL_NAME, joint names, file paths,
      START_POINT/END_POINT) MUST match your main.py / world .sdf. They're
      copied from your working main.py as of this writing - update both
      files together if you ever change the world/model.

USAGE
    python3 batch_test_runner.py
"""

import atexit
import csv
import datetime
import importlib
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

# ============================================================================
# Config copied from main.py - KEEP IN SYNC with your world/model setup.
# ============================================================================
WORLD_NAME = "water_test"
MODEL_NAME = "test_boat"
LEFT_JOINT = "left_thruster_joint"
RIGHT_JOINT = "right_thruster_joint"
BODY_YAW_OFFSET = 0.0  # test_boat spawns at yaw 0, local +X forward -> no correction

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

# ---- Drift-rejection terms (react to sideways wave push directly, instead
# of waiting for it to show up as heading error or accumulated cross-track
# position error) ----
# Kxte reacts to cross-track POSITION error (how far off the line you already
# are). A perpendicular wave hit shows up there late - by the time cross-track
# error has grown, the boat has already been pushed a long way sideways. The
# two gains below react earlier, directly to the boat's sideways motion:
#   KXTE_DOT   multiplies the (filtered) cross-track VELOCITY - how fast the
#              boat is currently drifting sideways. This is the primary term
#              to tune for "a beam wave shoves the boat but doesn't rotate
#              it": a sideways velocity spike shows up in this term almost
#              immediately, well before cross-track position error builds up,
#              so it can start countering the drift right away.
#   KXTE_DDOT  multiplies the (filtered) cross-track ACCELERATION - how
#              abruptly that sideways push is changing. This is the more
#              literal "acceleration pointing away from the robot" term, but
#              acceleration is a second derivative of noisy position data, so
#              it's inherently noisier/twitchier than the velocity term above.
#              Leave at 0.0 unless KXTE_DOT alone isn't reacting fast enough.
# Both are computed from a simple low-pass-filtered finite difference of
# cross-track position (see DRIFT_FILTER_TAU_DEFAULT below) so they don't
# just amplify simulation/sensor noise tick-to-tick.
KXTE_DOT_DEFAULT = 0.0
KXTE_DDOT_DEFAULT = 0.0
DRIFT_FILTER_TAU_DEFAULT = 0.5  # seconds; larger = smoother but more lag

TICK_S = 0.05  # ~20 Hz control loop

USE_WSL = False  # set True if gz sim runs in WSL but this script runs native Windows python

WAVE_TEMPLATE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf.template")
WAVE_REAL_SDF_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf")
WORLD_FILE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf")

WIND_SPEED_RANGE_DEFAULT = (2.0, 9.0)
WIND_ANGLE_RANGE_DEFAULT = (0.0, 360.0)
STEEPNESS_RANGE_DEFAULT = (0.5, 3.0)

POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
LEFT_TOPIC = f"/model/{MODEL_NAME}/joint/{LEFT_JOINT}/cmd_thrust"
RIGHT_TOPIC = f"/model/{MODEL_NAME}/joint/{RIGHT_JOINT}/cmd_thrust"

CSV_FIELDS = [
    "run", "seed", "wind_speed_mps", "wind_angle_deg", "steepness",
    "mode", "max_abs_cross_track_m", "final_abs_cross_track_m",
    "final_along_track_m", "arrived", "duration_s", "note",
]

# Per-tick time-series columns (one row per control-loop tick per phase).
TS_CSV_FIELDS = [
    "run", "mode", "t_s", "x_m", "y_m", "yaw_deg", "heading_error_deg",
    "cross_track_m", "cross_track_vel_mps", "cross_track_accel_mps2",
    "along_track_m", "left_thrust_N", "right_thrust_N",
]

# ============================================================================
# Small helpers (same math as main.py)
# ============================================================================


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
            return transport.Node, double_cls, posev_cls
        except ImportError:
            continue
    return None, None, None


NodeCls, DoubleMsg, PoseVMsg = load_gz_bindings()


def gz_cmd(args):
    return (["wsl", "--"] + args) if USE_WSL else args


def gz_service(service: str, reqtype: str, reptype: str, req: str, timeout_ms=2000):
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


# ============================================================================
# Wave randomization (same mechanism as main.py)
# ============================================================================


def randomize_and_write_wave_model(wind_speed_range, wind_angle_range, steepness_range, seed=None):
    if not WAVE_TEMPLATE_PATH.exists():
        print(f"Wave template not found at {WAVE_TEMPLATE_PATH} - skipping wave randomization.")
        return None
    if not WAVE_REAL_SDF_PATH.parent.exists():
        print(f"Wave model directory not found at {WAVE_REAL_SDF_PATH.parent} - skipping wave randomization.")
        return None

    backup_path = WAVE_REAL_SDF_PATH.with_suffix(WAVE_REAL_SDF_PATH.suffix + ".orig_bak")
    if WAVE_REAL_SDF_PATH.exists() and not backup_path.exists():
        shutil.copy2(WAVE_REAL_SDF_PATH, backup_path)

    used_seed = seed if seed is not None else random.SystemRandom().randrange(2 ** 31)
    rng = random.Random(used_seed)
    wind_speed = rng.uniform(*wind_speed_range)
    wind_angle = rng.uniform(*wind_angle_range)
    steepness = rng.uniform(*steepness_range)

    text = WAVE_TEMPLATE_PATH.read_text()
    text = text.replace("__WIND_SPEED__", f"{wind_speed:.3f}")
    text = text.replace("__WIND_ANGLE_DEG__", f"{wind_angle:.2f}")
    text = text.replace("__STEEPNESS__", f"{steepness:.3f}")
    if "__" in text:
        print("Warning: template still has an unfilled placeholder - check WAVE_TEMPLATE_PATH.")
    WAVE_REAL_SDF_PATH.write_text(text)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] seed={used_seed}  wind_speed={wind_speed:.3f} m/s  "
          f"wind_angle_deg={wind_angle:.2f}  steepness={steepness:.3f}")

    return used_seed, wind_speed, wind_angle, steepness


def patch_real_time_factor(new_rtf):
    """Temporarily override <real_time_factor> in the world file to speed
    the whole batch up. Returns a backup Path to restore from, or None if
    no change was made."""
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


# ============================================================================
# gz sim process management
# ============================================================================

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
    """WorldControl reset: teleports models back to their spawn pose and
    resets sim time to 0, WITHOUT restarting the gz sim process - so the
    wave field / wind conditions stay identical between the No-PID and PID
    phases of the same run."""
    req = "reset: {all: true}"
    result = gz_service(f"/world/{world_name}/control", "gz.msgs.WorldControl", "gz.msgs.Boolean", req)
    if getattr(result, "returncode", -1) != 0:
        print(f"  WARNING: world reset call failed: {getattr(result, 'stderr', '').strip()}")
    return result


# ============================================================================
# Pose tracking
# ============================================================================


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
            # after a world reset, wait until the pose actually reflects
            # the boat being back near the start point
            _, along = cross_along_track(state.x, state.y)
            if abs(along) < 0.5:
                return True
        if time.time() - start > timeout:
            return state.x is not None
        time.sleep(0.1)


# ============================================================================
# Control loop for one phase (no_pid / pid)
# ============================================================================


def run_phase(state, pub_left, pub_right, mode, cfg, run_idx):
    """Runs one phase (no_pid or pid) and returns (metrics dict, records
    list). `records` holds one dict per control-loop tick with position,
    heading, cross/along-track error and thrust - used for the time-series
    plots. heading_error is computed (and logged) in BOTH modes, even
    though only the pid mode actually uses it for control, so the two
    modes can be compared on the same time-series plots.

    DRIFT (BEAM-WAVE) REJECTION: besides the heading-error PID and the
    cross-track POSITION term (Kxte), this also estimates how fast the boat
    is currently being pushed sideways - its cross-track VELOCITY - and,
    optionally, how abruptly that push is changing - cross-track
    ACCELERATION. A wave hitting the boat from the side can shove it
    sideways a lot without rotating it much, so heading error alone reacts
    late; the velocity/acceleration terms let the controller start steering
    into the drift as soon as it starts, rather than waiting for it to turn
    into a large heading or position error. Both are derived from a simple
    low-pass-filtered finite difference of the cross-track position
    (time constant cfg["drift_tau"]) so tick-to-tick position noise doesn't
    get amplified into a jittery correction."""
    integral = 0.0
    prev_error = 0.0
    max_abs_cross = 0.0
    last_cross = 0.0
    along = 0.0
    start_time = time.time()
    last_tick = start_time
    arrived = False
    target_yaw = LINE_HEADING
    records = []

    # Drift-estimation state (filtered cross-track velocity/acceleration).
    prev_cross = None
    filt_cross_vel = 0.0
    prev_filt_cross_vel = 0.0
    filt_cross_accel = 0.0
    drift_tau = cfg.get("drift_tau", DRIFT_FILTER_TAU_DEFAULT)

    while True:
        now = time.time()
        dt = now - last_tick if now > last_tick else TICK_S
        last_tick = now
        elapsed = now - start_time

        if elapsed > cfg["timeout_s"]:
            break

        if state.x is None:
            time.sleep(TICK_S)
            continue

        cross, along = cross_along_track(state.x, state.y)
        max_abs_cross = max(max_abs_cross, abs(cross))
        # Tracked on every tick (even the final one, which may break out
        # below before being appended to `records`) so callers can report
        # the FINAL path divergence at the end of the run, not just the
        # worst-case divergence seen at any point during it.
        last_cross = cross

        if along >= LINE_LENGTH - GOAL_RADIUS:
            arrived = True
            break

        heading_error = wrap_pi(target_yaw - state.yaw) if state.yaw is not None else None

        # ---- Estimate filtered cross-track velocity/acceleration --------
        # (computed every tick, in both modes, so no_pid vs pid plots stay
        # comparable - only pid mode actually feeds it back into thrust).
        raw_cross_vel = 0.0 if prev_cross is None else (cross - prev_cross) / dt if dt > 0 else 0.0
        prev_cross = cross
        alpha_v = dt / (drift_tau + dt) if (drift_tau + dt) > 0 else 1.0
        prev_filt_cross_vel = filt_cross_vel
        filt_cross_vel += alpha_v * (raw_cross_vel - filt_cross_vel)

        raw_cross_accel = (filt_cross_vel - prev_filt_cross_vel) / dt if dt > 0 else 0.0
        alpha_a = alpha_v  # reuse the same time-constant-derived smoothing factor
        filt_cross_accel += alpha_a * (raw_cross_accel - filt_cross_accel)

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

        lmsg, rmsg = DoubleMsg(), DoubleMsg()
        lmsg.data, rmsg.data = left, right
        pub_left.publish(lmsg)
        pub_right.publish(rmsg)

        print(f"\r  run {run_idx} [{mode:6s}] t={elapsed:6.1f}s along={along:6.2f}/{LINE_LENGTH:.1f}m "
              f"cross={cross:+6.2f}m (max {max_abs_cross:.2f}m)   ", end="", flush=True)

        time.sleep(TICK_S)

    # stop thrusters at the end of the phase
    lmsg, rmsg = DoubleMsg(), DoubleMsg()
    lmsg.data = rmsg.data = 0.0
    pub_left.publish(lmsg)
    pub_right.publish(rmsg)
    print()

    duration = time.time() - start_time
    metrics = {
        "max_abs_cross_track_m": max_abs_cross,
        "final_abs_cross_track_m": abs(last_cross),
        "final_along_track_m": along,
        "arrived": arrived,
        "duration_s": duration,
    }
    return metrics, records


# ============================================================================
# CSV logging
# ============================================================================


def append_csv_row(csv_path, run_idx, seed, wind_speed, wind_angle, steepness, mode,
                    max_abs_cross_track_m=None, final_abs_cross_track_m=None,
                    final_along_track_m=None,
                    arrived=None, duration_s=None, note=""):
    write_header = not os.path.exists(csv_path)

    def fmt(v, digits=4):
        return f"{v:.{digits}f}" if isinstance(v, float) and v == v else ("" if v is None else v)

    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow({
            "run": run_idx,
            "seed": seed,
            "wind_speed_mps": fmt(wind_speed, 3),
            "wind_angle_deg": fmt(wind_angle, 2),
            "steepness": fmt(steepness, 3),
            "mode": mode,
            "max_abs_cross_track_m": fmt(max_abs_cross_track_m, 4),
            "final_abs_cross_track_m": fmt(final_abs_cross_track_m, 4),
            "final_along_track_m": fmt(final_along_track_m, 4),
            "arrived": arrived,
            "duration_s": fmt(duration_s, 2),
            "note": note,
        })


def append_timeseries_rows(csv_path, run_idx, mode, records):
    """Appends one row per tick from run_phase()'s `records` list. Header
    is written once, the first time this file is touched in the batch (the
    file is deleted at the start of main() so this is safe)."""
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


# ============================================================================
# One full run: wave randomization -> launch -> no_pid -> reset -> pid -> kill
# ============================================================================


def run_single(run_idx, run_seed, cfg, csv_path, ts_csv_path):
    print(f"\n=== Run {run_idx}/{cfg['num_runs']}  (seed={run_seed}) ===")

    wave_result = randomize_and_write_wave_model(
        cfg["wind_speed_range"], cfg["wind_angle_range"], cfg["steepness_range"], seed=run_seed
    )
    if wave_result:
        used_seed, wind_speed, wind_angle, steepness = wave_result
    else:
        used_seed, wind_speed, wind_angle, steepness = run_seed, float("nan"), float("nan"), float("nan")

    proc = launch_world_background(headless=cfg["headless"])
    if proc is None:
        print(f"  Skipping run {run_idx}: could not launch gz sim.")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "no_pid", note="launch_failed")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid", note="launch_failed")
        return

    time.sleep(cfg["gz_boot_wait_s"])

    node = NodeCls()
    pub_left = node.advertise(LEFT_TOPIC, DoubleMsg)
    pub_right = node.advertise(RIGHT_TOPIC, DoubleMsg)
    state = PoseState()
    ok = node.subscribe(PoseVMsg, POSE_TOPIC, state.callback)
    if not ok:
        print(f"  WARNING: failed to subscribe to {POSE_TOPIC}")

    if not wait_for_pose(state, timeout=15.0):
        print(f"  Run {run_idx}: never received pose data on {POSE_TOPIC} - aborting this run.")
        terminate_gz_process(proc)
        time.sleep(1.0)
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "no_pid", note="no_pose_data")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid", note="no_pose_data")
        return

    # ---- Phase 1: No PID (constant thrust baseline) -----------------------
    metrics_no_pid, records_no_pid = run_phase(state, pub_left, pub_right, "no_pid", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "no_pid", **metrics_no_pid)
    append_timeseries_rows(ts_csv_path, run_idx, "no_pid", records_no_pid)
    print(f"  no_pid: max|cross|={metrics_no_pid['max_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_no_pid['arrived']}  t={metrics_no_pid['duration_s']:.1f}s")

    # ---- Reset (same process, same waves, boat back at spawn) -------------
    print(f"  Resetting world '{WORLD_NAME}' before PID phase...")
    reset_world(WORLD_NAME)
    time.sleep(1.5)
    wait_for_pose(state, timeout=5.0, want_reset_near_start=True)

    # ---- Phase 2: PID -------------------------------------------------------
    metrics_pid, records_pid = run_phase(state, pub_left, pub_right, "pid", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid", **metrics_pid)
    append_timeseries_rows(ts_csv_path, run_idx, "pid", records_pid)
    print(f"  pid:    max|cross|={metrics_pid['max_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_pid['arrived']}  t={metrics_pid['duration_s']:.1f}s")

    terminate_gz_process(proc)
    time.sleep(1.0)


def run_single_pid_only(run_idx, run_seed, cfg, csv_path, ts_csv_path):
    """Same as run_single(), but for PID TUNING MODE: only the PID phase is
    run (no baseline no_pid phase, no mid-run reset needed since the boat
    starts fresh at spawn for every run anyway). This is faster to iterate
    on than the full compare mode, since every run is spent exercising the
    controller you're actually tuning."""
    print(f"\n=== [PID TUNING] Run {run_idx}/{cfg['num_runs']}  (seed={run_seed}) ===")

    wave_result = randomize_and_write_wave_model(
        cfg["wind_speed_range"], cfg["wind_angle_range"], cfg["steepness_range"], seed=run_seed
    )
    if wave_result:
        used_seed, wind_speed, wind_angle, steepness = wave_result
    else:
        used_seed, wind_speed, wind_angle, steepness = run_seed, float("nan"), float("nan"), float("nan")

    proc = launch_world_background(headless=cfg["headless"])
    if proc is None:
        print(f"  Skipping run {run_idx}: could not launch gz sim.")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid", note="launch_failed")
        return

    time.sleep(cfg["gz_boot_wait_s"])

    node = NodeCls()
    pub_left = node.advertise(LEFT_TOPIC, DoubleMsg)
    pub_right = node.advertise(RIGHT_TOPIC, DoubleMsg)
    state = PoseState()
    ok = node.subscribe(PoseVMsg, POSE_TOPIC, state.callback)
    if not ok:
        print(f"  WARNING: failed to subscribe to {POSE_TOPIC}")

    if not wait_for_pose(state, timeout=15.0):
        print(f"  Run {run_idx}: never received pose data on {POSE_TOPIC} - aborting this run.")
        terminate_gz_process(proc)
        time.sleep(1.0)
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid", note="no_pose_data")
        return

    # ---- PID phase only -----------------------------------------------------
    metrics_pid, records_pid = run_phase(state, pub_left, pub_right, "pid", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid", **metrics_pid)
    append_timeseries_rows(ts_csv_path, run_idx, "pid", records_pid)
    print(f"  pid:    max|cross|={metrics_pid['max_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_pid['arrived']}  t={metrics_pid['duration_s']:.1f}s")

    terminate_gz_process(proc)
    time.sleep(1.0)


def run_single_xte_compare(run_idx, run_seed, cfg, csv_path, ts_csv_path):
    """XTE-ABLATION COMPARE MODE: runs the PID controller twice against the
    SAME wave field (same reset trick as run_single()) - once with the
    cross-track feedback terms (Kxte, Kxte_dot, Kxte_ddot) exactly as
    configured ("pid_xte"), and once with all three of them forced to 0.0,
    i.e. a heading-only PID ("pid_no_xte"). Kp/Ki/Kd and base thrust are
    identical in both phases - only the cross-track feedback is switched
    off, so any difference in behavior between the two phases is
    attributable to those terms."""
    print(f"\n=== [XTE COMPARE] Run {run_idx}/{cfg['num_runs']}  (seed={run_seed}) ===")

    wave_result = randomize_and_write_wave_model(
        cfg["wind_speed_range"], cfg["wind_angle_range"], cfg["steepness_range"], seed=run_seed
    )
    if wave_result:
        used_seed, wind_speed, wind_angle, steepness = wave_result
    else:
        used_seed, wind_speed, wind_angle, steepness = run_seed, float("nan"), float("nan"), float("nan")

    proc = launch_world_background(headless=cfg["headless"])
    if proc is None:
        print(f"  Skipping run {run_idx}: could not launch gz sim.")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid_xte", note="launch_failed")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid_no_xte", note="launch_failed")
        return

    time.sleep(cfg["gz_boot_wait_s"])

    node = NodeCls()
    pub_left = node.advertise(LEFT_TOPIC, DoubleMsg)
    pub_right = node.advertise(RIGHT_TOPIC, DoubleMsg)
    state = PoseState()
    ok = node.subscribe(PoseVMsg, POSE_TOPIC, state.callback)
    if not ok:
        print(f"  WARNING: failed to subscribe to {POSE_TOPIC}")

    if not wait_for_pose(state, timeout=15.0):
        print(f"  Run {run_idx}: never received pose data on {POSE_TOPIC} - aborting this run.")
        terminate_gz_process(proc)
        time.sleep(1.0)
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid_xte", note="no_pose_data")
        append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                        "pid_no_xte", note="no_pose_data")
        return

    # ---- Phase 1: PID WITH cross-track terms, exactly as configured -------
    metrics_xte, records_xte = run_phase(state, pub_left, pub_right, "pid_xte", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid_xte", **metrics_xte)
    append_timeseries_rows(ts_csv_path, run_idx, "pid_xte", records_xte)
    print(f"  pid_xte:    max|cross|={metrics_xte['max_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_xte['arrived']}  t={metrics_xte['duration_s']:.1f}s")

    # ---- Reset (same process, same waves, boat back at spawn) -------------
    print(f"  Resetting world '{WORLD_NAME}' before no-XTE phase...")
    reset_world(WORLD_NAME)
    time.sleep(1.5)
    wait_for_pose(state, timeout=5.0, want_reset_near_start=True)

    # ---- Phase 2: PID with cross-track terms forced OFF --------------------
    cfg_no_xte = dict(cfg)
    cfg_no_xte["kxte"] = 0.0
    cfg_no_xte["kxte_dot"] = 0.0
    cfg_no_xte["kxte_ddot"] = 0.0
    metrics_no_xte, records_no_xte = run_phase(state, pub_left, pub_right, "pid_no_xte", cfg_no_xte, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid_no_xte", **metrics_no_xte)
    append_timeseries_rows(ts_csv_path, run_idx, "pid_no_xte", records_no_xte)
    print(f"  pid_no_xte: max|cross|={metrics_no_xte['max_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_no_xte['arrived']}  t={metrics_no_xte['duration_s']:.1f}s")

    terminate_gz_process(proc)
    time.sleep(1.0)


def run_single_full_real(run_idx, run_seed, cfg, csv_path, ts_csv_path):
    """FULL REAL MODE: runs all THREE controller configurations against the
    SAME wave field for this run (same reset-between-phases trick as
    run_single() / run_single_xte_compare()), so every configuration is
    compared under identical wind/wave conditions:
        1. "no_pid"     - constant thrust, no controller at all (baseline).
        2. "pid_xte"    - the heading PID WITH cross-track feedback terms
                           (Kxte, Kxte_dot, Kxte_ddot) exactly as configured.
        3. "pid_no_xte" - the SAME heading PID (same Kp/Ki/Kd/base thrust)
                           but with all three cross-track feedback terms
                           forced to 0.0.
    This is the union of the No-PID-vs-PID comparison and the XTE-ablation
    comparison, run in a single pass so all three curves are directly
    comparable against the same randomized wave conditions, instead of
    being generated from two separate batches (and therefore two separate
    sets of random wave draws)."""
    print(f"\n=== [FULL REAL] Run {run_idx}/{cfg['num_runs']}  (seed={run_seed}) ===")

    wave_result = randomize_and_write_wave_model(
        cfg["wind_speed_range"], cfg["wind_angle_range"], cfg["steepness_range"], seed=run_seed
    )
    if wave_result:
        used_seed, wind_speed, wind_angle, steepness = wave_result
    else:
        used_seed, wind_speed, wind_angle, steepness = run_seed, float("nan"), float("nan"), float("nan")

    proc = launch_world_background(headless=cfg["headless"])
    if proc is None:
        print(f"  Skipping run {run_idx}: could not launch gz sim.")
        for m in ("no_pid", "pid_xte", "pid_no_xte"):
            append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                            m, note="launch_failed")
        return

    time.sleep(cfg["gz_boot_wait_s"])

    node = NodeCls()
    pub_left = node.advertise(LEFT_TOPIC, DoubleMsg)
    pub_right = node.advertise(RIGHT_TOPIC, DoubleMsg)
    state = PoseState()
    ok = node.subscribe(PoseVMsg, POSE_TOPIC, state.callback)
    if not ok:
        print(f"  WARNING: failed to subscribe to {POSE_TOPIC}")

    if not wait_for_pose(state, timeout=15.0):
        print(f"  Run {run_idx}: never received pose data on {POSE_TOPIC} - aborting this run.")
        terminate_gz_process(proc)
        time.sleep(1.0)
        for m in ("no_pid", "pid_xte", "pid_no_xte"):
            append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                            m, note="no_pose_data")
        return

    # ---- Phase 1: No PID (constant thrust baseline) -----------------------
    metrics_no_pid, records_no_pid = run_phase(state, pub_left, pub_right, "no_pid", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "no_pid", **metrics_no_pid)
    append_timeseries_rows(ts_csv_path, run_idx, "no_pid", records_no_pid)
    print(f"  no_pid:     max|cross|={metrics_no_pid['max_abs_cross_track_m']:.2f} m  "
          f"final|cross|={metrics_no_pid['final_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_no_pid['arrived']}  t={metrics_no_pid['duration_s']:.1f}s")

    # ---- Reset before PID-with-XTE phase -----------------------------------
    print(f"  Resetting world '{WORLD_NAME}' before pid_xte phase...")
    reset_world(WORLD_NAME)
    time.sleep(1.5)
    wait_for_pose(state, timeout=5.0, want_reset_near_start=True)

    # ---- Phase 2: PID WITH cross-track terms, exactly as configured -------
    metrics_xte, records_xte = run_phase(state, pub_left, pub_right, "pid_xte", cfg, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid_xte", **metrics_xte)
    append_timeseries_rows(ts_csv_path, run_idx, "pid_xte", records_xte)
    print(f"  pid_xte:    max|cross|={metrics_xte['max_abs_cross_track_m']:.2f} m  "
          f"final|cross|={metrics_xte['final_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_xte['arrived']}  t={metrics_xte['duration_s']:.1f}s")

    # ---- Reset before PID-without-XTE phase --------------------------------
    print(f"  Resetting world '{WORLD_NAME}' before pid_no_xte phase...")
    reset_world(WORLD_NAME)
    time.sleep(1.5)
    wait_for_pose(state, timeout=5.0, want_reset_near_start=True)

    # ---- Phase 3: PID with cross-track terms forced OFF --------------------
    cfg_no_xte = dict(cfg)
    cfg_no_xte["kxte"] = 0.0
    cfg_no_xte["kxte_dot"] = 0.0
    cfg_no_xte["kxte_ddot"] = 0.0
    metrics_no_xte, records_no_xte = run_phase(state, pub_left, pub_right, "pid_no_xte", cfg_no_xte, run_idx)
    append_csv_row(csv_path, run_idx, used_seed, wind_speed, wind_angle, steepness,
                    "pid_no_xte", **metrics_no_xte)
    append_timeseries_rows(ts_csv_path, run_idx, "pid_no_xte", records_no_xte)
    print(f"  pid_no_xte: max|cross|={metrics_no_xte['max_abs_cross_track_m']:.2f} m  "
          f"final|cross|={metrics_no_xte['final_abs_cross_track_m']:.2f} m  "
          f"arrived={metrics_no_xte['arrived']}  t={metrics_no_xte['duration_s']:.1f}s")

    terminate_gz_process(proc)
    time.sleep(1.0)

# Where the last-used config-dialog values are remembered between runs of
# this script. Lives next to the script itself (not the CWD), so it's found
# the same way no matter where you launch python3 from.
CONFIG_SAVE_PATH = Path(__file__).resolve().parent / "batch_test_runner_last_config.json"

# Raw field keys (as used by the `fields` dict in show_config_dialog, plus
# "mode" and "headless") that get persisted to CONFIG_SAVE_PATH. Kept as a
# single source of truth so save/load can't drift out of sync with the
# dialog's own field list.
_PERSISTED_KEYS = [
    "mode", "runs", "seed", "timeout", "boot", "base", "kp", "ki", "kd", "kxte",
    "kxte_dot", "kxte_ddot", "drift_tau",
    "ws_min", "ws_max", "wa_min", "wa_max", "st_min", "st_max", "rtf",
    "csv", "ts_csv", "plot", "headless",
]


def load_last_config():
    """Returns a dict of last-saved raw field values (all strings, plus a
    bool for "headless"), or {} if there's no saved config yet / it's
    unreadable. Never raises - a corrupt/missing file just means the
    dialog falls back to its hardcoded defaults."""
    if not CONFIG_SAVE_PATH.exists():
        return {}
    try:
        import json
        with open(CONFIG_SAVE_PATH, "r") as f:
            data = json.load(f)
        return {k: v for k, v in data.items() if k in _PERSISTED_KEYS}
    except Exception as e:
        print(f"Could not read saved config at {CONFIG_SAVE_PATH} ({e}) - using defaults.")
        return {}


def save_last_config(values):
    """`values` is a dict with the same keys as _PERSISTED_KEYS (raw
    strings from the Entry widgets, plus "mode" and "headless"). Best
    effort - a failure to save should never block starting the batch."""
    try:
        import json
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

    def sv(key, hardcoded_default):
        """Saved value for `key` if we have one, else the hardcoded default."""
        return saved[key] if key in saved else hardcoded_default

    root = tk.Tk()
    root.title("Autonomous Boat Test — Batch Config")

    fields = {}

    def add_row(r, label, key, default):
        ttk.Label(root, text=label).grid(row=r, column=0, sticky="w", padx=8, pady=3)
        v = tk.StringVar(value=str(sv(key, default)))
        ttk.Entry(root, textvariable=v, width=18).grid(row=r, column=1, padx=8, pady=3)
        return v

    # ---- Mode selector: full No-PID vs PID comparison, or PID-only tuning --
    ttk.Label(root, text="Mode").grid(row=0, column=0, sticky="w", padx=8, pady=3)
    mode_var = tk.StringVar(value=sv("mode", "compare"))
    mode_frame = ttk.Frame(root)
    mode_frame.grid(row=0, column=1, sticky="w", padx=8, pady=3)
    ttk.Radiobutton(mode_frame, text="Compare (No PID vs PID)", variable=mode_var,
                     value="compare").pack(anchor="w")
    ttk.Radiobutton(mode_frame, text="PID tuning (PID only + error plots)", variable=mode_var,
                     value="pid_tuning").pack(anchor="w")
    ttk.Radiobutton(mode_frame, text="XTE compare (PID with vs without cross-track terms)",
                     variable=mode_var, value="xte_compare").pack(anchor="w")
    ttk.Radiobutton(mode_frame, text="Full real (No PID vs PID w/XTE vs PID no XTE)",
                     variable=mode_var, value="full_real").pack(anchor="w")

    fields["runs"] = add_row(1, "Number of test runs", "runs", 5)
    fields["seed"] = add_row(2, "Batch seed (blank = fully random each run)", "seed", "")
    fields["timeout"] = add_row(3, "Per-phase timeout (s)", "timeout", 90)
    fields["boot"] = add_row(4, "gz sim boot wait (s)", "boot", 3.0)
    fields["base"] = add_row(5, "Base thrust (N)", "base", BASE_THRUST_DEFAULT)
    fields["kp"] = add_row(6, "PID Kp", "kp", KP_DEFAULT)
    fields["ki"] = add_row(7, "PID Ki", "ki", KI_DEFAULT)
    fields["kd"] = add_row(8, "PID Kd", "kd", KD_DEFAULT)
    fields["kxte"] = add_row(9, "PID Kxte (cross-track position)", "kxte", KXTE_DEFAULT)
    fields["kxte_dot"] = add_row(10, "PID Kxte_dot (cross-track drift rate)", "kxte_dot", KXTE_DOT_DEFAULT)
    fields["kxte_ddot"] = add_row(11, "PID Kxte_ddot (cross-track drift accel, optional)", "kxte_ddot", KXTE_DDOT_DEFAULT)
    fields["drift_tau"] = add_row(12, "Drift filter time constant (s)", "drift_tau", DRIFT_FILTER_TAU_DEFAULT)
    fields["ws_min"] = add_row(13, "Wind speed min (m/s)", "ws_min", WIND_SPEED_RANGE_DEFAULT[0])
    fields["ws_max"] = add_row(14, "Wind speed max (m/s)", "ws_max", WIND_SPEED_RANGE_DEFAULT[1])
    fields["wa_min"] = add_row(15, "Wind angle min (deg)", "wa_min", WIND_ANGLE_RANGE_DEFAULT[0])
    fields["wa_max"] = add_row(16, "Wind angle max (deg)", "wa_max", WIND_ANGLE_RANGE_DEFAULT[1])
    fields["st_min"] = add_row(17, "Steepness min", "st_min", STEEPNESS_RANGE_DEFAULT[0])
    fields["st_max"] = add_row(18, "Steepness max", "st_max", STEEPNESS_RANGE_DEFAULT[1])
    fields["rtf"] = add_row(19, "Real-time factor override (1.0 = no change)", "rtf", 1.0)
    fields["csv"] = add_row(20, "Output summary CSV path", "csv", "batch_results.csv")
    fields["ts_csv"] = add_row(21, "Output time-series CSV path", "ts_csv", "batch_timeseries.csv")
    fields["plot"] = add_row(22, "Output plot path (base name)", "plot", "batch_divergence_plot.png")

    headless_var = tk.BooleanVar(value=bool(sv("headless", True)))
    ttk.Checkbutton(root, text="Run gz sim headless (server only — much faster)",
                    variable=headless_var).grid(row=23, column=0, columnspan=2, sticky="w", padx=8, pady=(6, 0))

    note = ""
    ttk.Label(root, text=note, foreground="#555", wraplength=380, justify="left").grid(
        row=24, column=0, columnspan=2, sticky="w", padx=8, pady=(6, 0)
    )

    def on_start():
        try:
            result["mode"] = mode_var.get()
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
            result["wind_speed_range"] = (float(fields["ws_min"].get()), float(fields["ws_max"].get()))
            result["wind_angle_range"] = (float(fields["wa_min"].get()), float(fields["wa_max"].get()))
            result["steepness_range"] = (float(fields["st_min"].get()), float(fields["st_max"].get()))
            result["real_time_factor"] = float(fields["rtf"].get())
            result["output_csv"] = fields["csv"].get().strip() or "batch_results.csv"
            result["output_timeseries_csv"] = fields["ts_csv"].get().strip() or "batch_timeseries.csv"
            result["output_plot"] = fields["plot"].get().strip() or "batch_divergence_plot.png"
            result["headless"] = bool(headless_var.get())
        except ValueError:
            messagebox.showerror("Bad input", "Please check the values (numbers only; runs >= 1).")
            return

        # Persist the raw form values (not the parsed `result`) so the
        # dialog can be pre-filled exactly as typed next time, including a
        # blank seed field.
        to_save = {key: v.get() for key, v in fields.items()}
        to_save["mode"] = mode_var.get()
        to_save["headless"] = bool(headless_var.get())
        save_last_config(to_save)

        started["ok"] = True
        root.destroy()

    def on_cancel():
        root.destroy()

    btn_frame = ttk.Frame(root)
    btn_frame.grid(row=25, column=0, columnspan=2, pady=10)
    ttk.Button(btn_frame, text="Start batch", command=on_start).pack(side="left", padx=6)
    ttk.Button(btn_frame, text="Cancel", command=on_cancel).pack(side="left", padx=6)

    root.mainloop()
    return result if started["ok"] else None


# ============================================================================
# Plotting
# ============================================================================


def _plot_path_with_suffix(out_path, suffix):
    """batch_divergence_plot.png + '_heading_time' -> batch_divergence_plot_heading_time.png"""
    p = Path(out_path)
    return str(p.with_name(p.stem + suffix + p.suffix))


def _load_summary(csv_path, modes=("no_pid", "pid"), value_col="max_abs_cross_track_m"):
    """Returns {mode: (runs, values)} for each mode in `modes`, where `runs`
    and `values` are parallel lists of run number / |value_col| (aborted
    runs with no data, or runs missing that column, are skipped). Defaults
    to the max |cross-track error| column; pass value_col=
    "final_abs_cross_track_m" to get the FINAL |cross-track error| instead."""
    result = {m: ([], []) for m in modes}
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row.get(value_col):
                continue  # skip aborted runs / runs with no data for this column
            mode = row["mode"]
            if mode not in result:
                continue
            run = int(row["run"])
            val = abs(float(row[value_col]))
            result[mode][0].append(run)
            result[mode][1].append(val)
    return result


def _load_timeseries(ts_csv_path, modes=("no_pid", "pid")):
    """Returns {mode: {run: {'t': [...], 'cross': [...], 'yaw': [...], 'along': [...]}}}
    for each mode in `modes`."""
    data = {m: {} for m in modes}
    if not os.path.exists(ts_csv_path):
        return data
    with open(ts_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
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
    """Like _load_timeseries(), but keeps the SIGNED cross-track error and
    heading error (not abs), plus thrust commands and the filtered
    cross-track drift velocity/acceleration, for PID-tuning diagnostic
    plots. Returns {run: {'t':[...], 'heading_error':[...],
    'cross_track':[...], 'cross_track_vel':[...], 'cross_track_accel':[...],
    'left_thrust':[...], 'right_thrust':[...]}}."""
    data = {}
    if not os.path.exists(ts_csv_path):
        return data
    with open(ts_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["mode"] != "pid":
                continue
            run = int(row["run"])
            bucket = data.setdefault(run, {
                "t": [], "heading_error": [], "cross_track": [],
                "cross_track_vel": [], "cross_track_accel": [],
                "left_thrust": [], "right_thrust": [],
            })
            bucket["t"].append(float(row["t_s"]))
            bucket["heading_error"].append(float(row["heading_error_deg"]))
            bucket["cross_track"].append(float(row["cross_track_m"]))
            bucket["cross_track_vel"].append(float(row.get("cross_track_vel_mps", 0.0) or 0.0))
            bucket["cross_track_accel"].append(float(row.get("cross_track_accel_mps2", 0.0) or 0.0))
            bucket["left_thrust"].append(float(row["left_thrust_N"]))
            bucket["right_thrust"].append(float(row["right_thrust_N"]))
    return data


def _mean_std_over_time(per_run_dict, key, grid):
    """per_run_dict: {run: {'t': [...], key: [...]}}. Interpolates every
    run's series onto `grid`, leaving NaN past that run's own end time
    (no extrapolation), then returns (mean, std) ignoring NaNs at each grid
    point. Returns (None, None) if there's no data at all."""
    import numpy as np

    if not per_run_dict:
        return None, None
    stacked = []
    for run, series in per_run_dict.items():
        t = np.asarray(series["t"])
        v = np.asarray(series[key])
        if len(t) < 2:
            continue
        interp = np.interp(grid, t, v, left=np.nan, right=np.nan)
        stacked.append(interp)
    if not stacked:
        return None, None
    stacked = np.vstack(stacked)
    with np.errstate(all="ignore"):
        mean = np.nanmean(stacked, axis=0)
        std = np.nanstd(stacked, axis=0)
    return mean, std


def _plot_mean_std_over_time(ts_data, key, ylabel, title, out_path,
                              modes=("no_pid", "pid"),
                              colors=None, labels=None):
    """One figure comparing the given modes: mean line + shaded +/-1 std
    band over time, built from all runs' time-series for `key`. `ts_data`
    must have an entry for every mode in `modes` (as returned by
    _load_timeseries(..., modes=modes))."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if colors is None:
        colors = {"no_pid": "crimson", "pid": "royalblue"}
    if labels is None:
        labels = {"no_pid": "No PID", "pid": "PID"}

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
        color = colors[mode]
        plt.plot(grid, mean, color=color, label=labels[mode], linewidth=2)
        plt.fill_between(grid, mean - std, mean + std, color=color, alpha=0.2)

    if not plotted_any:
        plt.close()
        return False

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title, fontsize=11)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved plot to {out_path}")
    return True


def _plot_divergence_scatter(csv_path, out_path, title, value_col, ylabel,
                              modes, colors, labels, markers=None):
    """Shared scatter-plot helper for a per-run divergence metric (max or
    final |cross-track error|) across an arbitrary set of modes. Used by
    both the 2-mode (No PID vs PID / XTE compare) and 3-mode (full real)
    plotting functions so the max-divergence and final-divergence scatters
    stay visually consistent with each other."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    if markers is None:
        default_markers = ["o", "^", "s", "D", "v", "P"]
        markers = {m: default_markers[i % len(default_markers)] for i, m in enumerate(modes)}

    summary = _load_summary(csv_path, modes=modes, value_col=value_col)

    fig, ax = plt.subplots(figsize=(9, 6))
    plotted_any = False
    for mode in modes:
        runs, vals = summary[mode]
        if not runs:
            continue
        plotted_any = True
        ax.scatter(runs, vals, color=colors[mode], label=labels[mode],
                   marker=markers[mode], s=60)
    ax.set_xlabel("Run number")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=11)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if plotted_any:
        ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {out_path}")
    return plotted_any


def make_plot(csv_path, ts_csv_path, out_path):
    modes = ("no_pid", "pid")
    colors = {"no_pid": "crimson", "pid": "royalblue"}
    labels = {"no_pid": "No PID", "pid": "PID"}
    markers = {"no_pid": "o", "pid": "^"}

    # ---- Plot 1: scatter of max |cross-track divergence| per run ----------
    _plot_divergence_scatter(
        csv_path, out_path,
        "Path divergence: No PID vs PID (across randomized wave conditions)",
        "max_abs_cross_track_m", "Max |cross-track divergence| (m)",
        modes, colors, labels, markers,
    )

    # ---- Plots 2-4: mean +/- std time-series comparisons -------------------
    ts_data = _load_timeseries(ts_csv_path, modes=modes)

    _plot_mean_std_over_time(
        ts_data, "cross", "|Cross-track error| (m)",
        "Mean |cross-track error| over time (shaded = ±1 std across runs)",
        _plot_path_with_suffix(out_path, "_cross_track_error_time"),
        modes=modes,
    )
    _plot_mean_std_over_time(
        ts_data, "yaw", "Heading / yaw (deg)",
        "Mean heading over time (shaded = ±1 std across runs)",
        _plot_path_with_suffix(out_path, "_heading_time"),
        modes=modes,
    )
    _plot_mean_std_over_time(
        ts_data, "along", "Along-track progress (m)",
        "Mean along-track progress over time (shaded = ±1 std across runs)",
        _plot_path_with_suffix(out_path, "_progress_time"),
        modes=modes,
    )


def make_xte_compare_plot(csv_path, ts_csv_path, out_path, cfg):
    """XTE-ABLATION COMPARE MODE plots: same structure as make_plot(), but
    comparing "pid_xte" (PID with cross-track terms as configured) against
    "pid_no_xte" (identical Kp/Ki/Kd, cross-track terms forced to 0) instead
    of No-PID vs PID. Produces the same 4 figures: a scatter of max
    |cross-track divergence| per run, plus mean +/- std over time for
    |cross-track error|, heading, and along-track progress."""
    modes = ("pid_no_xte", "pid_xte")
    colors = {"pid_no_xte": "darkorange", "pid_xte": "royalblue"}
    labels = {"pid_no_xte": "PID, no XTE terms", "pid_xte": "PID, with XTE terms"}
    markers = {"pid_no_xte": "o", "pid_xte": "^"}

    gains_str = (f"Kxte={cfg['kxte']:g}  Kxte_dot={cfg.get('kxte_dot', 0.0):g}  "
                 f"Kxte_ddot={cfg.get('kxte_ddot', 0.0):g}")

    # ---- Plot 1: scatter of max |cross-track divergence| per run ----------
    _plot_divergence_scatter(
        csv_path, out_path,
        f"Path divergence: PID with vs without cross-track terms\n({gains_str})",
        "max_abs_cross_track_m", "Max |cross-track divergence| (m)",
        modes, colors, labels, markers,
    )

    # ---- Plots 2-4: mean +/- std time-series comparisons -------------------
    ts_data = _load_timeseries(ts_csv_path, modes=modes)

    _plot_mean_std_over_time(
        ts_data, "cross", "|Cross-track error| (m)",
        f"Mean |cross-track error| over time — with vs without XTE terms\n({gains_str})",
        _plot_path_with_suffix(out_path, "_cross_track_error_time"),
        modes=modes, colors=colors, labels=labels,
    )
    _plot_mean_std_over_time(
        ts_data, "yaw", "Heading / yaw (deg)",
        f"Mean heading over time — with vs without XTE terms\n({gains_str})",
        _plot_path_with_suffix(out_path, "_heading_time"),
        modes=modes, colors=colors, labels=labels,
    )
    _plot_mean_std_over_time(
        ts_data, "along", "Along-track progress (m)",
        f"Mean along-track progress over time — with vs without XTE terms\n({gains_str})",
        _plot_path_with_suffix(out_path, "_progress_time"),
        modes=modes, colors=colors, labels=labels,
    )


def make_full_real_plot(csv_path, ts_csv_path, out_path, cfg):
    """FULL REAL MODE plots: three-way comparison of "no_pid", "pid_no_xte"
    and "pid_xte", all drawn from the SAME batch of runs / wave conditions.
    Produces everything make_plot()/make_xte_compare_plot() produce (a
    scatter of MAX |cross-track divergence| per run, plus mean +/- std over
    time for |cross-track error|, heading, and along-track progress) PLUS
    two additional figures:
      - a scatter of the FINAL |cross-track divergence| per run (i.e. how
        far off the line the boat ended up at the end of the run, as
        opposed to the worst moment during it).
      - a PID-only (pid_no_xte vs pid_xte) mean +/- std |cross-track error|
        plot, EXCLUDING "no_pid". The 3-way cross-track plot above is
        dominated by how much larger no_pid's divergence is, which flattens
        the with-XTE vs without-XTE difference down to barely-visible scale
        (see the 3-way plot vs the xte_compare-mode plot for the same
        gains). This extra figure re-plots just the two PID variants
        against each other so that difference is actually readable."""
    modes = ("no_pid", "pid_no_xte", "pid_xte")
    colors = {"no_pid": "crimson", "pid_no_xte": "darkorange", "pid_xte": "royalblue"}
    labels = {"no_pid": "No PID", "pid_no_xte": "PID, no XTE terms", "pid_xte": "PID, with XTE terms"}
    markers = {"no_pid": "o", "pid_no_xte": "s", "pid_xte": "^"}

    gains_str = (f"Kp={cfg['kp']:g}  Ki={cfg['ki']:g}  Kd={cfg['kd']:g}  "
                 f"Kxte={cfg['kxte']:g}  Kxte_dot={cfg.get('kxte_dot', 0.0):g}  "
                 f"Kxte_ddot={cfg.get('kxte_ddot', 0.0):g}")

    # ---- Plot 1: scatter of MAX |cross-track divergence| per run ----------
    _plot_divergence_scatter(
        csv_path, out_path,
        f"Path divergence (max): No PID vs PID (no XTE) vs PID (XTE)\n[{gains_str}]",
        "max_abs_cross_track_m", "Max |cross-track divergence| (m)",
        modes, colors, labels, markers,
    )

    # ---- Additional plot: scatter of FINAL |cross-track divergence| -------
    # Distinct from the "max" scatter above: this is where the boat ended
    # up when the phase finished (arrival or timeout), not the worst
    # moment of divergence seen at any point along the way.
    _plot_divergence_scatter(
        csv_path, _plot_path_with_suffix(out_path, "_final_divergence"),
        f"Path divergence (final): No PID vs PID (no XTE) vs PID (XTE)\n[{gains_str}]",
        "final_abs_cross_track_m", "Final |cross-track divergence| (m)",
        modes, colors, labels, markers,
    )

    # ---- Plots 3-5: mean +/- std time-series comparisons (3-way) -----------
    ts_data = _load_timeseries(ts_csv_path, modes=modes)

    _plot_mean_std_over_time(
        ts_data, "cross", "|Cross-track error| (m)",
        f"Mean |cross-track error| over time — No PID vs PID (no XTE) vs PID (XTE)\n[{gains_str}]",
        _plot_path_with_suffix(out_path, "_cross_track_error_time"),
        modes=modes, colors=colors, labels=labels,
    )
    _plot_mean_std_over_time(
        ts_data, "yaw", "Heading / yaw (deg)",
        f"Mean heading over time — No PID vs PID (no XTE) vs PID (XTE)\n[{gains_str}]",
        _plot_path_with_suffix(out_path, "_heading_time"),
        modes=modes, colors=colors, labels=labels,
    )
    _plot_mean_std_over_time(
        ts_data, "along", "Along-track progress (m)",
        f"Mean along-track progress over time — No PID vs PID (no XTE) vs PID (XTE)\n[{gains_str}]",
        _plot_path_with_suffix(out_path, "_progress_time"),
        modes=modes, colors=colors, labels=labels,
    )

    # ---- Extra plot: PID-only comparison (with vs without XTE terms) -------
    # Same mean +/- std cross-track curve as above, but excluding "no_pid"
    # so the y-axis isn't dominated by its much larger divergence - this is
    # what actually shows the XTE-term effect at a readable scale (matches
    # the equivalent plot from xte_compare mode, but drawn from this same
    # full_real batch/wave conditions instead of a separate batch).
    xte_only_modes = ("pid_no_xte", "pid_xte")
    _plot_mean_std_over_time(
        ts_data, "cross", "|Cross-track error| (m)",
        f"Mean |cross-track error| over time — PID (no XTE) vs PID (XTE) only\n[{gains_str}]",
        _plot_path_with_suffix(out_path, "_xte_only_cross_track_error_time"),
        modes=xte_only_modes, colors=colors, labels=labels,
    )


def make_pid_tuning_plot(ts_csv_path, out_path, cfg):
    """PID-TUNING diagnostic plot: a single figure with stacked subplots,
    one run per color, showing (top to bottom):
        1. Heading error (deg) vs time  - the signal the PID is driving to
           zero. Watch for: steady-state offset (raise Ki), slow decay
           (raise Kp), overshoot/ringing (lower Kp or raise Kd), high-freq
           chatter (lower Kd or add filtering).
        2. Cross-track error (m) vs time - the resulting path-following
           error (also fed back via Kxte if you're using it).
        3. Cross-track drift velocity/acceleration vs time - the filtered
           sideways-drift signals fed back via Kxte_dot/Kxte_ddot. This is
           the one to watch for beam-wave pushes: a wave hitting the boat
           from the side shows up here as a velocity (and, more sharply, an
           acceleration) spike well before it turns into a large heading or
           cross-track-position error - if the boat isn't correcting for
           those pushes fast enough, raise Kxte_dot (and only reach for
           Kxte_ddot if that alone isn't reacting quickly enough).
        4. Left/right thruster commands (N) vs time - useful for spotting
           saturation (commands pinned at 0 or MAX_THRUST) and oscillation
           in the actuator output itself, which usually shows up here
           before it's obvious in the error traces.
    A dashed zero-line is drawn on the error/drift subplots as a reference.
    Individual runs are plotted (not just mean/std) since with only a
    handful of tuning runs you want to see each one's actual behavior,
    not a smoothed average of them - but if there are many runs, only
    the first few are labeled in the legend to avoid clutter.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ts_data = _load_timeseries_pid_tuning(ts_csv_path)
    if not ts_data:
        print("No PID time-series data found - skipping PID tuning plot.")
        return False

    fig, (ax_head, ax_cross, ax_drift, ax_thrust) = plt.subplots(
        4, 1, figsize=(10, 14), sharex=True
    )

    cmap = plt.get_cmap("tab10")
    max_labeled = 10  # avoid an unreadable legend if there are many runs

    for i, (run_idx, series) in enumerate(sorted(ts_data.items())):
        color = cmap(i % 10)
        label = f"run {run_idx}" if i < max_labeled else None

        ax_head.plot(series["t"], series["heading_error"], color=color, linewidth=1.4, label=label)
        ax_cross.plot(series["t"], series["cross_track"], color=color, linewidth=1.4, label=label)
        ax_drift.plot(series["t"], series["cross_track_vel"], color=color, linewidth=1.3,
                       linestyle="-", label=(f"{label} vel" if label else None))
        ax_drift.plot(series["t"], series["cross_track_accel"], color=color, linewidth=1.0,
                       linestyle="--", alpha=0.7, label=(f"{label} accel" if label else None))
        ax_thrust.plot(series["t"], series["left_thrust"], color=color, linewidth=1.2,
                        linestyle="-", label=(f"{label} L" if label else None))
        ax_thrust.plot(series["t"], series["right_thrust"], color=color, linewidth=1.2,
                        linestyle="--", label=(f"{label} R" if label else None))

    ax_head.axhline(0.0, color="black", linewidth=0.8, linestyle=":")
    ax_head.set_ylabel("Heading error (deg)")
    ax_head.set_title(
        "PID tuning diagnostics\n"
        f"Kp={cfg['kp']:g}  Ki={cfg['ki']:g}  Kd={cfg['kd']:g}  "
        f"Kxte={cfg['kxte']:g}  Kxte_dot={cfg.get('kxte_dot', 0.0):g}  "
        f"Kxte_ddot={cfg.get('kxte_ddot', 0.0):g}  (base thrust={cfg['base_thrust']:g} N)",
        fontsize=11,
    )
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


# ============================================================================
# Main
# ============================================================================


def main():
    if NodeCls is None:
        print(
            "gz-transport Python bindings not found. Install e.g.:\n"
            "  sudo apt install python3-gz-transport14\n"
            "(or python3-gz-transport12 / 13 / 15, matching your gz version)\n"
            "This batch runner needs live pose feedback and cannot run without them."
        )
        return

    cfg = show_config_dialog()
    if cfg is None:
        print("Cancelled.")
        return

    print("Batch config:")
    for k, v in cfg.items():
        print(f"  {k}: {v}")

    # Reset output files at the START of each batch so a previous batch's
    # leftover rows never get appended to / mixed into this batch's plots
    # (this was the cause of duplicate run numbers and broken graphs).
    for path_key in ("output_csv", "output_timeseries_csv"):
        p = Path(cfg[path_key])
        if p.exists():
            print(f"Removing previous output file: {p}")
            p.unlink()

    seed_master = random.Random(cfg["seed"]) if cfg["seed"] is not None else random.SystemRandom()

    run_fn_by_mode = {
        "pid_tuning": run_single_pid_only,
        "xte_compare": run_single_xte_compare,
        "full_real": run_single_full_real,
        "compare": run_single,
    }
    mode = cfg.get("mode", "compare")
    run_fn = run_fn_by_mode.get(mode, run_single)

    rtf_backup = patch_real_time_factor(cfg["real_time_factor"])
    try:
        for run_idx in range(1, cfg["num_runs"] + 1):
            run_seed = seed_master.randrange(2 ** 31)
            try:
                run_fn(run_idx, run_seed, cfg, cfg["output_csv"], cfg["output_timeseries_csv"])
            except Exception as e:
                print(f"  ERROR during run {run_idx}: {e}")
                terminate_gz_process(_active_gz_proc)
    finally:
        restore_real_time_factor(rtf_backup)

    print(f"\nAll runs complete. Summary results in {cfg['output_csv']}")
    print(f"Time-series results in {cfg['output_timeseries_csv']}")
    try:
        if mode == "pid_tuning":
            make_pid_tuning_plot(
                cfg["output_timeseries_csv"],
                _plot_path_with_suffix(cfg["output_plot"], "_pid_tuning"),
                cfg,
            )
        elif mode == "xte_compare":
            make_xte_compare_plot(
                cfg["output_csv"], cfg["output_timeseries_csv"],
                _plot_path_with_suffix(cfg["output_plot"], "_xte_compare"),
                cfg,
            )
        elif mode == "full_real":
            make_full_real_plot(
                cfg["output_csv"], cfg["output_timeseries_csv"],
                _plot_path_with_suffix(cfg["output_plot"], "_full_real"),
                cfg,
            )
        else:
            make_plot(cfg["output_csv"], cfg["output_timeseries_csv"], cfg["output_plot"])
    except ImportError:
        print("matplotlib/numpy not installed - install with:\n  pip install matplotlib numpy --break-system-packages")


if __name__ == "__main__":
    main()
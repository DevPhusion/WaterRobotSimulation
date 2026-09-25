#!/usr/bin/env python3
"""
Autonomous "drive the line" control + live parameter tuning for the gz-sim
dual-thruster boat, with:

  - AUTO-LAUNCH (default): randomizes the wave field (wind speed, wind
    direction, steepness) within configurable ranges, writes them into the
    real waves model .sdf file, and starts `gz sim` itself in the
    background - so every run of this script is a fresh, different sea
    state, and you don't need a separate launcher script or a manually
    pre-started sim. Pass --no-auto-launch to skip this and just connect to
    a gz sim you already started yourself.

  - TWO NAV MODES, selectable in the UI before pressing Start:
      * PID    - heading-hold PID controller tracking the fixed
                 START_POINT -> END_POINT line (as before).
      * Normal - no controller at all: both thrusters run at the same
                 constant base-thrust value for the whole run. This is the
                 baseline to compare PID against - run the same (or
                 different) wave conditions through both modes and compare
                 the cross-track/heading logs.

Both modes: hold/drive the same fixed line, compute cross-track error
(perpendicular distance from the line) and along-track progress every tick,
auto-stop on arrival at the end marker, and can optionally log every tick to
CSV (now tagged with a "mode" column) for offline comparison.

Low latency: opens ONE persistent gz-transport connection at startup
(publishers + a pose subscriber) and reuses it for every tick. Needs the
Python bindings package installed:

    sudo apt install python3-gz-transport14   # or 13/12/15, see below

If it can't find a working bindings install, it prints a warning and falls
back to the old subprocess-per-call method for manual sliders only - the
nav loop needs live pose feedback and won't run without the bindings.

The param-tuning "Apply" button still uses the `gz service` CLI directly
(subprocess), since that's a rare, one-off action.

Requires the `gz` CLI on PATH.

USAGE
    python3 main.py                          # random waves, auto-launches gz sim
    python3 main.py --no-auto-launch         # attach to an already-running sim
    python3 main.py --seed 42                # reproducible wave conditions
    python3 main.py --wind-speed 3 9 --wind-angle 0 360 --steepness 0.5 2.5
"""

import argparse
import csv
import datetime
import importlib
import math
import os
import random
import shutil
import signal
import subprocess
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

# ---- Edit to match your world/model -------------------------------------
WORLD_NAME = "water_test"
MODEL_NAME = "test_boat"
LEFT_JOINT = "left_thruster_joint"
RIGHT_JOINT = "right_thruster_joint"
SPAWN_POSE = "0 0 0.1 0 0 0"    # x y z roll pitch yaw
HULL_SIZE = (1.0, 0.5, 0.2)     # x y z, must match collision/visual box

# ---- PID test path --------------------------------------------------------
# MUST match the <pose> of start_marker / end_marker in the .world file.
START_POINT = (0.0, 0.0)    # x, y (metres)
END_POINT = (20.0, 0.0)     # x, y (metres)
GOAL_RADIUS = 1.0           # metres; "arrived" tolerance around END_POINT

_dx = END_POINT[0] - START_POINT[0]
_dy = END_POINT[1] - START_POINT[1]
LINE_LENGTH = math.hypot(_dx, _dy)
LINE_HEADING = math.atan2(_dy, _dx)                       # radians
LINE_DIR = (_dx / LINE_LENGTH, _dy / LINE_LENGTH)          # unit vector A->B

MAX_THRUST = 60.0        # N, safety clamp
BASE_THRUST_DEFAULT = 35.0
KP_DEFAULT = 40.0
KI_DEFAULT = 0.0
KD_DEFAULT = 8.0
KXTE_DEFAULT = 0.0       # optional cross-track feedback gain (N per metre); 0 = off
TICK_MS = 50               # ~20 Hz control loop

# Set True if running this script with native Windows Python while gz-sim
# runs inside WSL. Routes subprocess `gz` calls (service calls, world
# auto-launch, and the fallback publish path) through `wsl`. Leave False if
# you're running with WSL's python3 directly (the common case).
USE_WSL = False
# ---------------------------------------------------------------------------

# ---- Wave randomization + auto-launch (domain randomization for PID testing) ----
AUTO_LAUNCH_WORLD = True   # default: randomize waves + start gz sim automatically

# Keep these in sync with launch_randomized_world.py if you still use it
# standalone (e.g. to pre-stage conditions and then run this with
# --no-auto-launch).
WAVE_TEMPLATE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf.template")
WAVE_REAL_SDF_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf")
WORLD_FILE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/backupOption.sdf")

# wind_speed intentionally stays clear of 0.26-0.39 m/s, a known asv_wave_sim
# FFT instability band (upstream issue #172) - don't lower the minimum into it.
WIND_SPEED_RANGE = (2.0, 9.0)      # m/s
WIND_ANGLE_RANGE = (0.0, 360.0)    # degrees
STEEPNESS_RANGE = (0.5, 3.0)

GZ_BOOT_WAIT_S = 2.0  # seconds to sleep after launching gz sim before connecting bindings

POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
LEFT_TOPIC = f"/model/{MODEL_NAME}/joint/{LEFT_JOINT}/cmd_thrust"
RIGHT_TOPIC = f"/model/{MODEL_NAME}/joint/{RIGHT_JOINT}/cmd_thrust"


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
    """Signed perpendicular distance (cross-track) and distance traveled
    along the line (along-track), relative to START_POINT -> END_POINT.

    Cross-track sign convention: positive = boat is to the LEFT of the line
    when facing from START_POINT toward END_POINT; negative = to the right.
    """
    px = x - START_POINT[0]
    py = y - START_POINT[1]
    along = px * LINE_DIR[0] + py * LINE_DIR[1]
    cross = -px * LINE_DIR[1] + py * LINE_DIR[0]
    return cross, along


def load_gz_bindings():
    """Try known gz-transport/gz-msgs version pairings, newest first.
    Returns (Node_class, Double_cls, PoseV_cls) or (None, None, None)."""
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


def gz_topic_pub_subprocess(topic: str, value: float) -> None:
    try:
        subprocess.run(
            gz_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {value}"]),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        print("Could not find 'gz' (or 'wsl'). Check PATH / USE_WSL setting.")


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


def box_inertia(mass: float, size):
    x, y, z = size
    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0
    return ixx, iyy, izz


def build_model_sdf(p: dict) -> str:
    ixx, iyy, izz = box_inertia(p["mass"], HULL_SIZE)
    x, y, z = HULL_SIZE
    sdf = f"""
<sdf version="1.9">
<model name="{MODEL_NAME}">
  <pose>{SPAWN_POSE}</pose>
  <link name="hull">
    <inertial>
      <mass>{p['mass']}</mass>
      <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision"><geometry><box><size>{x} {y} {z}</size></box></geometry></collision>
    <visual name="visual">
      <geometry><box><size>{x} {y} {z}</size></box></geometry>
      <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
    </visual>
  </link>
  <link name="left_prop">
    <pose relative_to="hull">-0.55 0.2 0 0 1.5708 0</pose>
    <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertial></inertial>
    <visual name="visual"><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
  </link>
  <joint name="{LEFT_JOINT}" type="revolute">
    <parent>hull</parent><child>left_prop</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
  </joint>
  <link name="right_prop">
    <pose relative_to="hull">-0.55 -0.2 0 0 1.5708 0</pose>
    <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertial></inertial>
    <visual name="visual"><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
  </link>
  <joint name="{RIGHT_JOINT}" type="revolute">
    <parent>hull</parent><child>right_prop</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
  </joint>
  <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
    <link_name>hull</link_name>
    <xU>{p['xU']}</xU><xUabsU>{p['xUabsU']}</xUabsU>
    <yV>{p['yV']}</yV><yVabsV>{p['yVabsV']}</yVabsV>
    <zW>{p['zW']}</zW><zWabsW>{p['zWabsW']}</zWabsW>
    <kP>{p['kP']}</kP><kPabsP>{p['kPabsP']}</kPabsP>
    <mQ>{p['mQ']}</mQ><mQabsQ>{p['mQabsQ']}</mQabsQ>
    <nR>{p['nR']}</nR><nRabsR>{p['nRabsR']}</nRabsR>
  </plugin>
  <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
    <joint_name>{LEFT_JOINT}</joint_name>
    <thrust_coefficient>{p['thrust_coefficient']}</thrust_coefficient>
    <fluid_density>{p['fluid_density']}</fluid_density>
    <propeller_diameter>{p['propeller_diameter']}</propeller_diameter>
  </plugin>
  <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
    <joint_name>{RIGHT_JOINT}</joint_name>
    <thrust_coefficient>{p['thrust_coefficient']}</thrust_coefficient>
    <fluid_density>{p['fluid_density']}</fluid_density>
    <propeller_diameter>{p['propeller_diameter']}</propeller_diameter>
  </plugin>
</model>
</sdf>
""".strip()
    return sdf.replace("\n", " ").replace('"', '\\"')


def respawn(p: dict) -> str:
    remove_req = f'name: "{MODEL_NAME}" type: MODEL'
    r1 = gz_service(f"/world/{WORLD_NAME}/remove", "gz.msgs.Entity", "gz.msgs.Boolean", remove_req)

    sdf_inline = build_model_sdf(p)
    create_req = f'sdf: "{sdf_inline}"'
    r2 = gz_service(f"/world/{WORLD_NAME}/create", "gz.msgs.EntityFactory", "gz.msgs.Boolean", create_req)

    return (
        f"remove: rc={r1.returncode} {r1.stdout.strip()} {r1.stderr.strip()}\n"
        f"create: rc={r2.returncode} {r2.stdout.strip()} {r2.stderr.strip()}"
    )


PARAM_FIELDS = [
    ("mass", "1.0"),
    ("thrust_coefficient", "0.005"),
    ("fluid_density", "1000"),
    ("propeller_diameter", "0.08"),
    ("xU", "-5"), ("xUabsU", "-10"),
    ("yV", "-10"), ("yVabsV", "-20"),
    ("zW", "-10"), ("zWabsW", "-20"),
    ("kP", "-4"), ("kPabsP", "-8"),
    ("mQ", "-4"), ("mQabsQ", "-8"),
    ("nR", "-2"), ("nRabsR", "-4"),
]


# ---- Wave randomization + world auto-launch --------------------------------
def randomize_and_write_wave_model(wind_speed_range=WIND_SPEED_RANGE, wind_angle_range=WIND_ANGLE_RANGE,
                                    steepness_range=STEEPNESS_RANGE, seed=None):
    """Pick random wave params and write them into the real waves model .sdf
    file. The gz-waves plugin only reads these at world load, so this MUST
    run before gz sim starts. Returns (seed, wind_speed, wind_angle,
    steepness), or None if the template/model paths aren't set up yet."""
    if not WAVE_TEMPLATE_PATH.exists():
        print(f"Wave template not found at {WAVE_TEMPLATE_PATH} - skipping wave randomization.")
        return None
    if not WAVE_REAL_SDF_PATH.parent.exists():
        print(f"Wave model directory not found at {WAVE_REAL_SDF_PATH.parent} - skipping wave randomization.")
        return None

    backup_path = WAVE_REAL_SDF_PATH.with_suffix(WAVE_REAL_SDF_PATH.suffix + ".orig_bak")
    if WAVE_REAL_SDF_PATH.exists() and not backup_path.exists():
        shutil.copy2(WAVE_REAL_SDF_PATH, backup_path)
        print(f"Backed up existing wave model to {backup_path}")

    used_seed = seed if seed is not None else random.SystemRandom().randrange(2**31)
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
    summary = (f"[{stamp}] seed={used_seed}  wind_speed={wind_speed:.3f} m/s  "
               f"wind_angle_deg={wind_angle:.2f}  steepness={steepness:.3f}")
    print(summary)
    log_path = WAVE_REAL_SDF_PATH.parent.parent / "wave_run_log.txt"
    try:
        with open(log_path, "a") as f:
            f.write(summary + "\n")
    except OSError:
        pass

    return used_seed, wind_speed, wind_angle, steepness


def launch_world_background():
    """Starts gz sim as a background (non-blocking) process. Returns the
    Popen handle so it can be cleaned up when the GUI closes, or None if it
    couldn't be started."""
    if not WORLD_FILE_PATH.exists():
        print(f"World file not found at {WORLD_FILE_PATH} - not auto-launching gz sim.")
        return None
    cmd = gz_cmd(["gz", "sim", "-r", str(WORLD_FILE_PATH)])
    print(f"Auto-launching: {' '.join(cmd)}")
    kwargs = {}
    if os.name == "posix":
        kwargs["preexec_fn"] = os.setsid  # lets us kill the whole process group later
    try:
        return subprocess.Popen(cmd, **kwargs)
    except FileNotFoundError:
        print("Could not find 'gz' (or 'wsl'). Check PATH / USE_WSL setting - not auto-launching.")
        return None


def terminate_gz_process(proc):
    if proc is None:
        return
    try:
        if os.name == "posix":
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        else:
            proc.terminate()
    except (ProcessLookupError, PermissionError):
        pass
    except Exception as e:
        print(f"Could not terminate gz sim process cleanly: {e}")


class App(tk.Tk):
    def __init__(self, gz_process=None, wave_info=None):
        super().__init__()
        self.title(f"Boat control — {MODEL_NAME}")
        self._gz_process = gz_process
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.have_bindings = NodeCls is not None
        self.current_yaw = None       # radians, updated by pose subscriber
        self.current_x = None         # metres, updated by pose subscriber
        self.current_y = None         # metres, updated by pose subscriber
        self.target_yaw = None
        self.nav_active = False
        self.active_mode = None       # "pid" or "normal" while running
        self.nav_start_time = None
        self.integral = 0.0
        self.prev_error = 0.0
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.log_file = None
        self.log_writer = None

        if self.have_bindings:
            self.node = NodeCls()
            self.pub_left = self.node.advertise(LEFT_TOPIC, DoubleMsg)
            self.pub_right = self.node.advertise(RIGHT_TOPIC, DoubleMsg)
            ok = self.node.subscribe(PoseVMsg, POSE_TOPIC, self._on_pose)
            if not ok:
                print(f"Warning: failed to subscribe to {POSE_TOPIC}")
        else:
            print(
                "gz-transport Python bindings not found - falling back to slower "
                "subprocess publishing, and nav is disabled (no live pose "
                "feedback). Try: sudo apt install python3-gz-transport14 "
                "(or python3-gz-transport13 / 12 / 15, whichever matches your install)."
            )

        # --- status banners -------------------------------------------------
        banner = "fast (gz-transport bindings)" if self.have_bindings else "SLOW fallback (subprocess) - install bindings, see console"
        ttk.Label(self, text=f"Publish path: {banner}").grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=(8, 0))
        ttk.Label(
            self,
            text=(f"Path: ({START_POINT[0]:.1f}, {START_POINT[1]:.1f}) -> "
                  f"({END_POINT[0]:.1f}, {END_POINT[1]:.1f})   "
                  f"length {LINE_LENGTH:.1f} m   bearing {math.degrees(LINE_HEADING):.1f} deg")
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=8)
        if wave_info:
            ttk.Label(self, text=wave_info).grid(row=2, column=0, columnspan=2, sticky="w", padx=8)

        # --- Navigation panel ----------------------------------------------
        nav = ttk.LabelFrame(self, text="Navigation: PID vs Normal (no controller) comparison")
        nav.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=8)

        self.mode_var = tk.StringVar(value="pid")
        mode_frame = ttk.Frame(nav)
        mode_frame.grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.mode_radio_pid = ttk.Radiobutton(mode_frame, text="PID", variable=self.mode_var, value="pid")
        self.mode_radio_pid.pack(side="left")
        self.mode_radio_normal = ttk.Radiobutton(mode_frame, text="Normal (const thrust)", variable=self.mode_var, value="normal")
        self.mode_radio_normal.pack(side="left")

        self.nav_btn = ttk.Button(nav, text="Start", command=self.toggle_nav,
                                   state=("normal" if self.have_bindings else "disabled"))
        self.nav_btn.grid(row=0, column=1, padx=4, pady=4)

        self.kp_var = tk.StringVar(value=str(KP_DEFAULT))
        self.ki_var = tk.StringVar(value=str(KI_DEFAULT))
        self.kd_var = tk.StringVar(value=str(KD_DEFAULT))
        self.kxte_var = tk.StringVar(value=str(KXTE_DEFAULT))
        self.base_var = tk.StringVar(value=str(BASE_THRUST_DEFAULT))
        for i, (label, var) in enumerate([("Kp", self.kp_var), ("Ki", self.ki_var),
                                           ("Kd", self.kd_var), ("Kxte", self.kxte_var),
                                           ("Base thrust N", self.base_var)]):
            ttk.Label(nav, text=label).grid(row=0, column=2 + 2 * i, sticky="e")
            ttk.Entry(nav, textvariable=var, width=8).grid(row=0, column=3 + 2 * i, padx=(0, 6))
        # Note: Kp/Ki/Kd/Kxte are ignored in Normal mode - only "Base thrust N"
        # applies there, so both modes can be compared at the same base thrust.

        self.log_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(nav, text="Log to CSV", variable=self.log_var).grid(
            row=0, column=2 + 2 * 5, padx=(6, 0)
        )

        self.nav_status = tk.Label(
            nav, text="not running", width=78, height=4, justify="left",
            relief="sunken", bg="#222", fg="#0f0", font=("Courier", 10), anchor="w",
        )
        self.nav_status.grid(row=1, column=0, columnspan=13, padx=6, pady=6, sticky="w")

        # --- Manual thrust sliders (used only while nav is stopped) --------
        manual = ttk.LabelFrame(self, text="Manual thrust (only while nav is stopped)")
        manual.grid(row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        self.left_val = tk.DoubleVar()
        self.right_val = tk.DoubleVar()
        ttk.Label(manual, text="Left N").grid(row=0, column=0, sticky="w")
        self.left_scale = ttk.Scale(manual, from_=-MAX_THRUST, to=MAX_THRUST, variable=self.left_val,
                                     command=lambda _=None: self.set_thrust(self.left_val.get(), self.right_val.get()),
                                     length=300)
        self.left_scale.grid(row=1, column=0)
        ttk.Label(manual, text="Right N").grid(row=2, column=0, sticky="w")
        self.right_scale = ttk.Scale(manual, from_=-MAX_THRUST, to=MAX_THRUST, variable=self.right_val,
                                      command=lambda _=None: self.set_thrust(self.left_val.get(), self.right_val.get()),
                                      length=300)
        self.right_scale.grid(row=3, column=0)

        # --- Param tuning panel ----------------------------------------------
        param_frame = ttk.LabelFrame(self, text="Boat params (Apply = remove + respawn model)")
        param_frame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        self.entries = {}
        for i, (name, default) in enumerate(PARAM_FIELDS):
            ttk.Label(param_frame, text=name).grid(row=i, column=0, sticky="w")
            e = ttk.Entry(param_frame, width=12)
            e.insert(0, default)
            e.grid(row=i, column=1)
            self.entries[name] = e

        ttk.Button(self, text="Apply (respawn boat)", command=self.apply_params).grid(
            row=6, column=0, columnspan=2, pady=6
        )
        self.status = tk.Text(self, height=4, width=78)
        self.status.grid(row=7, column=0, columnspan=2, padx=8, pady=8)

        self.after(TICK_MS, self.tick)

    # ---- lifecycle ------------------------------------------------------
    def on_close(self):
        self._close_log()
        terminate_gz_process(self._gz_process)
        self.destroy()

    # ---- pose feedback (runs on gz-transport's callback thread) -------
    def _on_pose(self, msg):
        for pose in msg.pose:
            if pose.name == MODEL_NAME:
                q = pose.orientation
                self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
                self.current_x = pose.position.x
                self.current_y = pose.position.y
                return

    # ---- navigation -----------------------------------------------------
    def toggle_nav(self):
        if not self.nav_active:
            if self.current_yaw is None or self.current_x is None or self.current_y is None:
                messagebox.showinfo("Waiting for pose", "No pose data yet - is the sim running and unpaused?")
                return
            self.active_mode = self.mode_var.get()  # "pid" or "normal", locked in for this run
            # Hold the FIXED line bearing (start -> end), not just whatever
            # heading the boat happens to be facing right now.
            self.target_yaw = LINE_HEADING
            self.integral = 0.0
            self.prev_error = 0.0
            self.nav_start_time = time.time()
            self.nav_active = True
            self.nav_btn.config(text="Stop")
            self.left_scale.state(["disabled"])
            self.right_scale.state(["disabled"])
            self.mode_radio_pid.state(["disabled"])
            self.mode_radio_normal.state(["disabled"])
            self._open_log_if_requested()
        else:
            self._stop_nav("stopped by user")

    def _open_log_if_requested(self):
        if not self.log_var.get():
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(os.getcwd(), f"run_{self.active_mode}_{ts}.csv")
        try:
            self.log_file = open(path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "mode", "t_s", "x_m", "y_m", "yaw_deg", "target_yaw_deg",
                "heading_error_deg", "cross_track_m", "along_track_m",
                "left_thrust_N", "right_thrust_N",
            ])
            print(f"Logging {self.active_mode} run to {path}")
        except OSError as e:
            print(f"Could not open log file: {e}")
            self.log_file = None
            self.log_writer = None

    def _close_log(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None

    def _stop_nav(self, reason: str):
        self.nav_active = False
        self.nav_btn.config(text="Start")
        self.left_scale.state(["!disabled"])
        self.right_scale.state(["!disabled"])
        self.mode_radio_pid.state(["!disabled"])
        self.mode_radio_normal.state(["!disabled"])
        self.set_thrust(0.0, 0.0)
        self._close_log()
        mode_label = "PID" if self.active_mode == "pid" else "Normal"
        self.nav_status.config(text=f"[{mode_label}] stopped: {reason}")
        self.active_mode = None

    def run_control_tick(self, dt):
        cross, along = cross_along_track(self.current_x, self.current_y)

        # Arrived at the end marker -> stop (same criterion for both modes).
        if along >= LINE_LENGTH - GOAL_RADIUS:
            self._stop_nav(
                f"reached end point (along={along:.2f} m, cross-track={cross:+.2f} m)"
            )
            return

        try:
            base = float(self.base_var.get())
        except ValueError:
            self.nav_status.config(text="Bad base thrust value")
            return

        heading_error = wrap_pi(self.target_yaw - self.current_yaw)

        if self.active_mode == "normal":
            # No controller: both thrusters run at the same constant speed
            # the whole time, regardless of heading or cross-track drift.
            # This is the baseline PID is compared against.
            left = right = clamp(base, 0.0, MAX_THRUST)
        else:
            try:
                kp, ki, kd = float(self.kp_var.get()), float(self.ki_var.get()), float(self.kd_var.get())
                kxte = float(self.kxte_var.get())
            except ValueError:
                self.nav_status.config(text="Bad Kp/Ki/Kd/Kxte value")
                return
            self.integral += heading_error * dt
            derivative = (heading_error - self.prev_error) / dt if dt > 0 else 0.0
            self.prev_error = heading_error
            # Heading PID + optional cross-track feedback (line-of-sight
            # style). If the boat is left of the line (cross > 0), steer
            # right to converge back. Kxte defaults to 0 (pure heading-hold).
            correction = kp * heading_error + ki * self.integral + kd * derivative - kxte * cross
            # positive correction -> turn left -> more thrust on the right
            # side, less on the left.
            left = clamp(base - correction, 0.0, MAX_THRUST)
            right = clamp(base + correction, 0.0, MAX_THRUST)

        self.set_thrust(left, right)

        elapsed = time.time() - self.nav_start_time if self.nav_start_time else 0.0
        mode_label = "PID" if self.active_mode == "pid" else "Normal"
        self.nav_status.config(text=(
            f"[{mode_label}]  t={elapsed:6.1f}s  progress {along:6.2f} / {LINE_LENGTH:.1f} m\n"
            f"heading {math.degrees(self.current_yaw):7.1f} deg  target {math.degrees(self.target_yaw):7.1f} deg  "
            f"error {math.degrees(heading_error):+6.1f} deg\n"
            f"cross-track {cross:+6.2f} m (sign: + = left of line)\n"
            f"left {self.left_thrust:6.1f} N   right {self.right_thrust:6.1f} N"
        ))

        if self.log_writer is not None:
            self.log_writer.writerow([
                self.active_mode,
                f"{elapsed:.3f}", f"{self.current_x:.3f}", f"{self.current_y:.3f}",
                f"{math.degrees(self.current_yaw):.2f}", f"{math.degrees(self.target_yaw):.2f}",
                f"{math.degrees(heading_error):.2f}", f"{cross:.3f}", f"{along:.3f}",
                f"{self.left_thrust:.2f}", f"{self.right_thrust:.2f}",
            ])

    # ---- thrust output (fast path via bindings, fallback via subprocess) --
    def set_thrust(self, left, right):
        self.left_thrust = clamp(left, -MAX_THRUST, MAX_THRUST)
        self.right_thrust = clamp(right, -MAX_THRUST, MAX_THRUST)
        if self.have_bindings:
            lmsg, rmsg = DoubleMsg(), DoubleMsg()
            lmsg.data, rmsg.data = self.left_thrust, self.right_thrust
            self.pub_left.publish(lmsg)
            self.pub_right.publish(rmsg)
        else:
            gz_topic_pub_subprocess(LEFT_TOPIC, self.left_thrust)
            gz_topic_pub_subprocess(RIGHT_TOPIC, self.right_thrust)

    def tick(self):
        now = time.time()
        dt = now - getattr(self, "_last_tick", now)
        self._last_tick = now
        if self.nav_active and self.current_yaw is not None and self.current_x is not None:
            self.run_control_tick(dt if dt > 0 else TICK_MS / 1000.0)
        self.after(TICK_MS, self.tick)

    # ---- param tuning -------------------------------------------------
    def apply_params(self):
        try:
            p = {name: float(self.entries[name].get()) for name, _ in PARAM_FIELDS}
        except ValueError:
            messagebox.showerror("Bad value", "All fields must be numbers.")
            return
        result = respawn(p)
        self.status.delete("1.0", tk.END)
        self.status.insert(tk.END, result)


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-auto-launch", action="store_true",
                     help="Don't randomize waves / launch gz sim - assume it's already running")
    ap.add_argument("--seed", type=int, default=None,
                     help="Fix the wave-randomization RNG seed (default: random each run)")
    ap.add_argument("--wind-speed", type=float, nargs=2, metavar=("MIN", "MAX"), default=WIND_SPEED_RANGE)
    ap.add_argument("--wind-angle", type=float, nargs=2, metavar=("MIN", "MAX"), default=WIND_ANGLE_RANGE)
    ap.add_argument("--steepness", type=float, nargs=2, metavar=("MIN", "MAX"), default=STEEPNESS_RANGE)
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    gz_process = None
    wave_info = None

    if AUTO_LAUNCH_WORLD and not args.no_auto_launch:
        result = randomize_and_write_wave_model(
            tuple(args.wind_speed), tuple(args.wind_angle), tuple(args.steepness), seed=args.seed
        )
        gz_process = launch_world_background()
        if gz_process is not None:
            time.sleep(GZ_BOOT_WAIT_S)  # give gz sim a moment to come up before connecting
        if result:
            used_seed, wind_speed, wind_angle, steepness = result
            wave_info = (f"Waves this run: seed={used_seed}  wind_speed={wind_speed:.2f} m/s  "
                         f"wind_angle={wind_angle:.1f} deg  steepness={steepness:.2f}")
        else:
            wave_info = "Wave randomization skipped (see console) - using existing wave model."
    else:
        wave_info = "Auto-launch disabled (--no-auto-launch) - assuming gz sim & waves are already set up."

    App(gz_process=gz_process, wave_info=wave_info).mainloop()
#!/usr/bin/env python3
"""
Randomize the wave field (wind speed, wind direction, steepness) within
configurable ranges, write them into the real waves model .sdf file, then
launch gz sim.

WHY THIS EXISTS
The gz-waves1 plugin (both the WavesModel and WavesVisual copies of the
<wave> block) only reads wind_speed / wind_angle_deg / steepness once, at
world load. There's no live "set wave params" service wired up in this
version (the SDF template even has a `\todo populate from service instead`
comment marking that as unfinished). So to get a NEW random sea state you
regenerate the model file and restart the sim - which is also just the
standard way to do domain randomization for controller testing: vary
conditions PER RUN so the PID isn't tuned to one specific sea state.

This does NOT make waves change mid-run. If you also want that (waves
morphing while the boat is already driving), that needs a live parameter
service the plugin doesn't currently expose - flag it if you want to look
into that separately.

USAGE
    python3 launch_randomized_world.py                # random every run
    python3 launch_randomized_world.py --seed 42       # reproducible run
    python3 launch_randomized_world.py --dry-run        # write file, don't launch gz
    python3 launch_randomized_world.py --wind-speed 3 9 --wind-angle 0 360 --steepness 0.5 2.5
"""

import argparse
import datetime
import random
import shutil
import subprocess
from pathlib import Path

# ---- EDIT THESE THREE PATHS to match your machine -------------------------
# The template created alongside this script (waves.sdf.template).
TEMPLATE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf.template")

# The REAL model file gz-sim actually loads, inside the models/waves/
# directory referenced by <uri> in the world file. Gazebo model dirs
# conventionally use "model.sdf", but some (like this one, going by the
# filename you pasted) use "<model_name>.sdf" - check
# models/waves/model.config's <sdf> tag if you're not sure, then fix this
# path once.
REAL_MODEL_SDF_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/models/waves/model.sdf")

# The world file to launch after regenerating the wave model.
WORLD_FILE_PATH = Path("/home/phusion/gz_ws/worlds/CrestWaterRobot/water_test_with_markers.world")
# ---------------------------------------------------------------------------

# Default randomization ranges. wind_speed intentionally stays clear of
# 0.26-0.39 m/s, a known asv_wave_sim FFT instability band (upstream issue
# #172) - don't lower WIND_SPEED_RANGE's minimum into that band.
WIND_SPEED_RANGE = (2.0, 9.0)      # m/s
WIND_ANGLE_RANGE = (0.0, 360.0)    # degrees
STEEPNESS_RANGE = (0.5, 3.0)       # unitless, plugin-defined scale

# Set True if gz-sim runs inside WSL but this script runs under native
# Windows Python (same convention as boat_pid_control.py).
USE_WSL = False


def gz_cmd(args):
    return (["wsl", "--"] + args) if USE_WSL else args


def pick_values(rng: random.Random):
    wind_speed = rng.uniform(*WIND_SPEED_RANGE)
    wind_angle = rng.uniform(*WIND_ANGLE_RANGE)
    steepness = rng.uniform(*STEEPNESS_RANGE)
    return wind_speed, wind_angle, steepness


def render_template(wind_speed: float, wind_angle: float, steepness: float) -> str:
    text = TEMPLATE_PATH.read_text()
    text = text.replace("__WIND_SPEED__", f"{wind_speed:.3f}")
    text = text.replace("__WIND_ANGLE_DEG__", f"{wind_angle:.2f}")
    text = text.replace("__STEEPNESS__", f"{steepness:.3f}")
    if "__" in text:
        raise ValueError(
            "Template still has an unfilled placeholder after substitution - "
            "check TEMPLATE_PATH points at the right file."
        )
    return text


def main():
    global WIND_SPEED_RANGE, WIND_ANGLE_RANGE, STEEPNESS_RANGE

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=None,
                     help="Fix the RNG seed for a reproducible run (default: random each time)")
    ap.add_argument("--wind-speed", type=float, nargs=2, metavar=("MIN", "MAX"), default=WIND_SPEED_RANGE)
    ap.add_argument("--wind-angle", type=float, nargs=2, metavar=("MIN", "MAX"), default=WIND_ANGLE_RANGE)
    ap.add_argument("--steepness", type=float, nargs=2, metavar=("MIN", "MAX"), default=STEEPNESS_RANGE)
    ap.add_argument("--dry-run", action="store_true", help="Only regenerate the model file, don't launch gz sim")
    args = ap.parse_args()

    WIND_SPEED_RANGE = tuple(args.wind_speed)
    WIND_ANGLE_RANGE = tuple(args.wind_angle)
    STEEPNESS_RANGE = tuple(args.steepness)

    if not TEMPLATE_PATH.exists():
        raise SystemExit(f"Template not found: {TEMPLATE_PATH}\n"
                          f"Copy waves.sdf.template there, or fix TEMPLATE_PATH in this script.")
    if not REAL_MODEL_SDF_PATH.parent.exists():
        raise SystemExit(f"Model directory not found: {REAL_MODEL_SDF_PATH.parent}\n"
                          f"Fix REAL_MODEL_SDF_PATH in this script.")

    # One-time backup of whatever was there before, so you can restore the
    # original fixed values if you ever want to.
    backup_path = REAL_MODEL_SDF_PATH.with_suffix(REAL_MODEL_SDF_PATH.suffix + ".orig_bak")
    if REAL_MODEL_SDF_PATH.exists() and not backup_path.exists():
        shutil.copy2(REAL_MODEL_SDF_PATH, backup_path)
        print(f"Backed up existing model file to {backup_path}")

    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**31)
    rng = random.Random(seed)
    wind_speed, wind_angle, steepness = pick_values(rng)

    rendered = render_template(wind_speed, wind_angle, steepness)
    REAL_MODEL_SDF_PATH.write_text(rendered)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary = (
        f"[{stamp}] seed={seed}  "
        f"wind_speed={wind_speed:.3f} m/s  "
        f"wind_angle_deg={wind_angle:.2f}  "
        f"steepness={steepness:.3f}"
    )
    print(summary)

    # Append to a running log so you can correlate a sim run's conditions
    # with the CSV the PID controller logs (match by wall-clock time).
    log_path = REAL_MODEL_SDF_PATH.parent.parent / "wave_run_log.txt"
    with open(log_path, "a") as f:
        f.write(summary + "\n")

    if args.dry_run:
        print(f"Dry run: wrote {REAL_MODEL_SDF_PATH}, not launching gz sim.")
        return

    if not WORLD_FILE_PATH.exists():
        print(f"Warning: world file not found at {WORLD_FILE_PATH} - "
              f"model file was regenerated but not launching gz sim.")
        return

    print(f"Launching: gz sim -r {WORLD_FILE_PATH}")
    subprocess.run(gz_cmd(["gz", "sim", "-r", str(WORLD_FILE_PATH)]))


if __name__ == "__main__":
    main()
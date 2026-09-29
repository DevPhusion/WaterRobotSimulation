# Rescue Float Robot: Gazebo Simulation & PID Path-Following Tests

A Gazebo Sim (gz-sim) simulation of a dual-thruster rescue float robot operating on **Vietnamese lakes and rivers**. The project tests how well a heading PID controller (with optional cross-track feedback) holds a straight 20 m line under wave, wind and, most importantly, **water-current disturbances**, compared with an uncontrolled constant-thrust baseline.

**Key features**

- Dual-thruster boat model built from real CAD meshes (`rescue-float-robot`), with wave-aware hydrodynamics ([asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) / `gz-waves`).
- Environment presets based on Vietnamese lake/river data: lake, river (dry season), river (rain season), mixed, or custom.
- Water current with randomised or swept direction (`/ocean_current`), so you can see how the controller copes with following, head-on and beam currents.
- Autonomous headless batch testing: N randomised runs per mode, CSV logging, and automatic plots.
- Four test modes: No-PID vs PID, PID tuning, PID with/without cross-track terms, and a three-way comparison.

---

## Demo

<!--
  Put your demo video here. Options:
  1. GitHub: drag-and-drop the .mp4 into this file while editing on github.com and it will
     insert a hosted link automatically.
  2. YouTube: replace VIDEO_ID below.
  3. Local file: put it in docs/videos/ and link it (large files: use Git LFS).
-->

**Demo video:** _add link or embed here_

<!-- Example (YouTube thumbnail link):
[![Demo video](docs/images/demo_thumbnail.png)](https://www.youtube.com/watch?v=VIDEO_ID)
-->

<!-- Example (GIF preview):
![Demo](docs/videos/demo.gif)
-->

---

## Table of contents

1. [Project structure](#project-structure)
2. [Installation](#installation)
3. [Running on Windows with WSL](#running-on-windows-with-wsl)
4. [Configure paths](#configure-paths)
5. [Running the simulation](#running-the-simulation)
6. [Batch testing](#batch-testing)
7. [Environments](#environments)
8. [Results](#results)
9. [Troubleshooting](#troubleshooting)
10. [Known limitations](#known-limitations)
11. [Credits](#credits)

---

## Project structure

```
gz_ws/
└── worlds/
    └── CrestWaterRobot/
        ├── mainSimulation.sdf              # World: physics, wind, waves, markers, test_boat
        └── models/
            ├── waves/
            │   ├── model.sdf               # Generated (do not edit by hand)
            │   └── model.sdf.template      # Wave template with __WIND_SPEED__ etc. placeholders
            └── realRobot/rescue-float-robot/assets/   # CAD meshes (.stl)

main.py                    # Single run: launch the world and drive the boat
batch_test_runner.py       # Autonomous batch tester (GUI config -> runs -> CSV -> plots)
docs/
├── images/                # Result plots for this README
└── videos/                # Demo video / GIFs
```

> Adjust this tree to match your repository. The absolute paths used by the scripts are listed in [Configure paths](#configure-paths).

---

## Installation

Tested on **Ubuntu 24.04** (native or WSL2). Adjust package names if you use a different Gazebo release.

### 1. Gazebo Sim

Install the Gazebo release you use (Harmonic and Ionic are both available on Ubuntu 24.04). Example for Harmonic:

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic
```

Verify: `gz sim --version`

### 2. Wave plugin (asv_wave_sim / gz-waves)

The world uses `gz-waves1-waves-model-system`, `gz-waves1-waves-visual-system` and `gz-waves1-hydrodynamics-system`. Build and install [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) following its README, choosing the branch that matches your Gazebo release, then make sure the plugin libraries are discoverable:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:<path-to-asv_wave_sim>/install/lib
```

Add it to `~/.bashrc` so it persists.

### 3. Python dependencies

The scripts need the gz-transport and gz-msgs Python bindings matching your Gazebo release, plus Tkinter, matplotlib and numpy:

| Gazebo release | Bindings package |
|---|---|
| Harmonic | `python3-gz-transport13` |
| Ionic | `python3-gz-transport14` |
| Jetty | `python3-gz-transport15` |

```bash
sudo apt-get install -y python3-gz-transport13 python3-tk python3-pip   # swap 13 for your release
pip install matplotlib numpy --break-system-packages
```

The runner auto-detects transport versions 12 to 15.

---

## Running on Windows with WSL

Gazebo runs best inside **WSL2** (Ubuntu). The simplest setup is to run *everything* inside WSL.

### Option A: everything inside WSL (recommended)

1. In PowerShell (admin):
   ```powershell
   wsl --install -d Ubuntu-24.04
   wsl --update
   ```
2. Restart, open the Ubuntu terminal, and follow [Installation](#installation) inside it.
3. Make sure **WSLg** works (Windows 11 / recent Windows 10). The Gazebo GUI and the Tkinter config window open as normal Windows windows. Test with `gz sim shapes.sdf` or `python3 -c "import tkinter; tkinter.Tk()"`.
4. Keep the project inside the Linux filesystem (`~/gz_ws/...`), not `/mnt/c/...`. It is much faster.
5. Run the scripts from the WSL terminal ([see below](#running-the-simulation)). Leave `USE_WSL = False`.

### Option B: Python on Windows, Gazebo in WSL

If you run `batch_test_runner.py` with native Windows Python but Gazebo lives in WSL:

1. Set `USE_WSL = True` at the top of `batch_test_runner.py`. All `gz` commands are then prefixed with `wsl --`.
2. Update the path constants to point at the files as seen by the process that reads them (the script writes the wave `.sdf` itself, so Windows Python needs a path such as `\\wsl$\Ubuntu-24.04\home\<user>\gz_ws\...`).
3. gz-transport on Windows must still be able to reach Gazebo in WSL, which is fragile. **Option A avoids all of this.**

### GPU / rendering tips

- Headless batch runs (`gz sim -s`) need no GPU or display.
- For the GUI in WSL, update your GPU driver on Windows (WSL2 GPU support). If rendering fails, try `export LIBGL_ALWAYS_SOFTWARE=1`.

---

## Configure paths

Both scripts contain absolute paths that **must match your machine**. Edit the constants near the top of `main.py` and `batch_test_runner.py`:

| Constant | Meaning | Default in this repo |
|---|---|---|
| `WORLD_FILE_PATH` | World file | `/home/phusion/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf` |
| `WAVE_TEMPLATE_PATH` | Wave template with placeholders | `.../models/waves/model.sdf.template` |
| `WAVE_REAL_SDF_PATH` | Generated wave model (overwritten each run) | `.../models/waves/model.sdf` |
| `WORLD_NAME` | `<world name>` in the SDF | `water_test` |
| `MODEL_NAME` | Boat model name | `test_boat` |
| `START_POINT` / `END_POINT` | Test line; must match the marker poses in the SDF | `(0, 0)` → `(20, 0)` |

Also check the absolute `<uri>` paths inside `mainSimulation.sdf` (waves model and CAD meshes). Keep the constants in both scripts in sync with each other and with the world file.

The runner backs up the original wave file (`model.sdf.orig_bak`) the first time it writes it, and it restores the world's `real_time_factor` after the batch if you overrode it.

---

## Running the simulation

### Single run

```bash
python3 main.py
```

<!-- TODO: describe what main.py does in your version (GUI? PID on/off? wave randomisation?) -->

### Launching the world manually (optional)

```bash
gz sim -r ~/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf        # with GUI
gz sim -r -s ~/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf     # headless (server only)
```

Useful topics:

```bash
# Set a water current by hand (world frame, m/s)
gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 1.0, y: 0.3, z: 0'

# Thruster commands
gz topic -t /model/test_boat/joint/left_thruster_joint/cmd_thrust  -m gz.msgs.Double -p 'data: 2.5'
gz topic -t /model/test_boat/joint/right_thruster_joint/cmd_thrust -m gz.msgs.Double -p 'data: 2.5'
```

---

## Batch testing

```bash
python3 batch_test_runner.py
```

A configuration window opens (last-used values are remembered in `batch_test_runner_last_config.json`). Press **Start batch**. For every run the script:

1. Draws an environment (current speed/direction, waves) and writes the wave model.
2. Launches `gz sim` headless.
3. Runs the phases of the selected mode, resetting the world between phases so every phase sees the **same waves and the same current**.
4. Logs results to CSV and kills `gz sim` before the next run.

After the batch it prints a per-environment summary table and saves plots.

### Modes

| | **compare** | **pid_tuning** | **xte_compare** | **full_real** |
|---|---|---|---|---|
| Purpose | Does PID beat doing nothing? | Tune the gains | Do cross-track terms help? | Both, in one batch |
| Phases per run | no_pid → pid | pid | pid_xte → pid_no_xte | no_pid → pid_xte → pid_no_xte |

| Phase | Behaviour |
|---|---|
| `no_pid` | Constant equal thrust on both motors, no feedback |
| `pid` / `pid_xte` | Heading PID plus cross-track position, velocity and acceleration terms (Kxte, Kxte_dot, Kxte_ddot) |
| `pid_no_xte` | Same PID with Kxte, Kxte_dot and Kxte_ddot forced to 0 |

### Current direction convention

The angle is the direction the **water flows toward**, in the world frame, degrees counter-clockwise from +X. The boat travels along +X:

| Angle | Meaning |
|---|---|
| 0° | Following current |
| 90° / 270° | Beam current (largest cross-track disturbance) |
| 180° | Head-on current (hardest for progress) |

The dialog can draw directions **randomly** or **sweep** them evenly around the circle.

### Output files

| File | Content |
|---|---|
| `batch_results.csv` | One row per run and phase: environment, current, waves, max/final cross-track error, arrival, duration, note (`timeout` / `diverged`) |
| `batch_timeseries.csv` | One row per control tick: pose, heading, cross/along-track error, drift velocity/acceleration, thrust |
| `batch_divergence_plot_*.png` | Plots (see [Results](#results)) |

Output files are deleted at the start of each batch so results never mix.

### Safety limits

A phase ends early on arrival (along-track ≥ 19 m), on timeout (default 90 s), or as `diverged` if the boat is more than 60 m off the line or 60 m backwards.

---

## Environments

Values are based on measurements collected in *Thông số môi trường sông, hồ Việt Nam* (Vietnamese river/lake environment parameters report). Current-speed ranges are the target test ranges; the report's own measured values are lower for lakes (0.03–0.17 m/s at Hồ Tây) and top out around 1.8 m/s for modelled flood flow.

| Environment | Current | Wave wind speed | Steepness | Approx. wave height* |
|---|---|---|---|---|
| Lake | 0.05–0.30 m/s | 0.8–2.0 m/s | 0.10–0.30 | ~0.02–0.10 m |
| River, dry season | 0.5–1.2 m/s | 1.0–2.5 m/s | 0.10–0.40 | ~0.03–0.15 m |
| River, rain season | 2.0–3.0 m/s | 1.5–3.0 m/s | 0.20–0.50 | ~0.05–0.22 m |
| Mixed | random category per run | | | |
| Custom | dialog fields | dialog fields | dialog fields | |

\* Rule-of-thumb estimate (Hs ≈ 0.0246·U²), an upper bound for fetch-limited lakes and rivers. Check visually.

---

## Results

> Replace the placeholder images below with your own plots. Save them in `docs/images/` and keep the file names, or edit the links. Add a short caption (batch size, environment, gains used) under each one.

**Batch settings used for the results below**

| Item | Value |
|---|---|
| Mode | _e.g. full_real_ |
| Environment | _e.g. River, dry season_ |
| Runs | _e.g. 20_ |
| Current direction | _e.g. random 0–360°_ |
| Kp / Ki / Kd | _e.g. 3 / 0 / 1.2_ |
| Kxte / Kxte_dot / Kxte_ddot | _e.g. 1 / 1 / 0_ |
| Base thrust | _e.g. 2.5 N_ |
| Seed | _e.g. 42_ |

### Path divergence (max cross-track error per run)

![Max cross-track divergence](docs/images/batch_divergence_plot_full_real.png)

_Caption: add your observations here._

### Path divergence (final cross-track error per run)

![Final cross-track divergence](docs/images/batch_divergence_plot_full_real_final_divergence.png)

### Cross-track error over time (mean ± 1 std)

![Cross-track error over time](docs/images/batch_divergence_plot_full_real_cross_track_error_time.png)

### PID only: with vs without cross-track terms

![XTE-only comparison](docs/images/batch_divergence_plot_full_real_xte_only_cross_track_error_time.png)

### Heading over time (mean ± 1 std)

![Heading over time](docs/images/batch_divergence_plot_full_real_heading_time.png)

### Along-track progress over time (mean ± 1 std)

![Along-track progress](docs/images/batch_divergence_plot_full_real_progress_time.png)

### Effect of current direction

Radius is the max cross-track error; angle is the direction the water flows toward (0° = following, 180° = head-on, 90°/270° = beam).

![Polar plot vs current direction](docs/images/batch_divergence_plot_current_direction.png)

### PID tuning diagnostics

![PID tuning diagnostics](docs/images/batch_divergence_plot_pid_tuning.png)

### Summary table

Paste the console summary (or your own table) here:

| Environment | Mode | Runs | Arrived | Mean max \|xte\| (m) | Mean final \|xte\| (m) |
|---|---|---|---|---|---|
| _river_dry_ | _no_pid_ | | | | |
| _river_dry_ | _pid_no_xte_ | | | | |
| _river_dry_ | _pid_xte_ | | | | |

### Additional results / notes

_Add anything else here: extra environments, screenshots of the Gazebo scene, comparison to real-world tests, etc._

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `gz-transport Python bindings not found` | Install `python3-gz-transport<N>` matching your Gazebo release (see [Installation](#3-python-dependencies)). |
| Config window does not appear | Install `python3-tk`. In WSL make sure WSLg works, or export `DISPLAY`. |
| `World file not found` / `Wave template not found` | Update the path constants ([Configure paths](#configure-paths)). |
| Pose topic never received | Check `WORLD_NAME` / `MODEL_NAME`. Inspect topics with `gz topic -l`. Increase *gz sim boot wait* in the dialog. |
| Plugin `gz-waves1-...` fails to load | Set `GZ_SIM_SYSTEM_PLUGIN_PATH` to the asv_wave_sim install `lib` directory. |
| Console prints `[check] WARNING: boat drift ... only ...` | The hull's hydrodynamics plugin is ignoring `/ocean_current`, so current-based results are not valid. See [Known limitations](#known-limitations). |
| Leftover `gz sim` process after a crash | `pkill -f "gz sim"` |
| Wave `model.sdf` looks wrong after a crash | Restore from `model.sdf.orig_bak`, or re-run the batch (it regenerates from the template). |

---

## Known limitations

- **Current coupling is unverified.** The world assumes the hull's hydrodynamics plugin reacts to `/ocean_current`. The runner's first-run drift check tells you whether it actually does. If not, the current has to be applied another way (for example as a drag force via `ApplyLinkWrench`).
- **Wave heights are estimates.** Wave size is controlled through the wind speed and steepness of the FFT wave model, not set directly in metres.
- **Strong currents can exceed thrust.** At 2–3 m/s the boat may not be able to make headway or hold the line. These phases end as `diverged` or `timeout`, and their cross-track error is capped near the 60 m abort limit.
- **Single measured sites.** Many values in the environment report come from one location and one time, so they are not representative of every lake or river.
- **Wind in the world file** is a fixed gust model (mean 2 m/s) and is not randomised per environment.

---

## Credits

- [Gazebo Sim](https://gazebosim.org/) (Open Robotics)
- [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) wave and hydrodynamics plugins
- Robot CAD meshes exported with [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)
- Environment parameters: *Thông số môi trường sông, hồ Việt Nam* report (see the source list in the report itself)

<!-- Add: authors, team/lab, license, contact -->

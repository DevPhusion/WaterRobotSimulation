# PIDXTE: Rescue Float Robot Simulation

A Gazebo Sim simulation of a dual-thruster rescue float robot on **Vietnamese lakes and rivers**. It tests how well three controllers hold a straight 20 m line under waves, wind and water-current disturbances:

- **No controller**: constant equal thrust on both motors.
- **Heading PID**: PID on the yaw error only.
- **PIDXTE**: heading PID plus cross-track error (XTE) terms.

**Result:** over 450 trials (3 environments × 50 runs × 3 controllers), PIDXTE gave the smallest cross-track error in every environment, typically 2 to 3 times smaller than the heading-only PID. See [Results](#results).

---

## Table of contents

1. [Controllers](#controllers)
2. [Installation](#installation)
3. [Configure paths](#configure-paths)
4. [Running](#running)
5. [Batch testing](#batch-testing)
6. [Environments](#environments)
7. [Results](#results)
8. [Known limitations](#known-limitations)
9. [Credits](#credits)

---

## Controllers

Heading PID, with `e = ψ_target − ψ_current`; the output is added to one thruster and subtracted from the other:

$$u(t) = K_p\,e(t) + K_i\int_0^t e(\tau)\,d\tau + K_d\,\frac{de(t)}{dt}$$

PIDXTE subtracts three cross-track terms from the PID output:

$$u(t) = K_p\,e_\theta + K_i\int_0^t e_\theta\,dt + K_d\,\dot e_\theta - \left(K_s\,e_d + K_v\,\dot e_d + K_a\,\ddot e_d\right)$$

$$e_d = \mathbf{n}\cdot(\mathbf{p}-\mathbf{s}), \qquad \dot e_d = \mathbf{n}\cdot\mathbf{v}, \qquad \ddot e_d = \mathbf{n}\cdot\mathbf{a}$$

**p**, **v**, **a** are the robot's position, velocity and acceleration, **s** is the path start, and **n** is the unit normal to the path (pointing left). With all XTE gains at 0, PIDXTE equals the heading PID.

| Report | Code | Term |
|---|---|---|
| Ks | `Kxte` | Cross-track position |
| Kv | `Kxte_dot` | Cross-track velocity |
| Ka | `Kxte_ddot` | Cross-track acceleration |

---

## Installation

Results were produced with **Gazebo Sim 7.9.0 on Ubuntu 22.04** (ODE physics). The steps below target Ubuntu 24.04 (native or WSL2).

**1. Gazebo Sim** (example: Harmonic)

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

**2. Wave plugin.** Build [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) (branch matching your Gazebo release) and add it to the plugin path (put it in `~/.bashrc`):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:<path-to-asv_wave_sim>/install/lib
```

**3. Python dependencies**

| Gazebo release | Bindings package |
|---|---|
| Garden | `python3-gz-transport12` |
| Harmonic | `python3-gz-transport13` |
| Ionic | `python3-gz-transport14` |
| Jetty | `python3-gz-transport15` |

```bash
sudo apt-get install -y python3-gz-transport13 python3-tk python3-pip   # swap 13 for your release
pip install matplotlib numpy --break-system-packages
```

**Windows:** run everything inside WSL2 (Ubuntu) with WSLg, keep the project in the Linux filesystem (`~/gz_ws/...`), and leave `USE_WSL = False`. Headless runs (`gz sim -s`) need no GPU or display.

---

## Configure paths

Edit the constants at the top of `main.py` and `batch_test_runner.py` (keep both in sync with the world file):

| Constant | Default |
|---|---|
| `WORLD_FILE_PATH` | `/home/phusion/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf` |
| `WAVE_TEMPLATE_PATH` | `.../models/waves/model.sdf.template` |
| `WAVE_REAL_SDF_PATH` | `.../models/waves/model.sdf` (generated each run) |
| `WORLD_NAME` / `MODEL_NAME` | `water_test` / `test_boat` |
| `START_POINT` / `END_POINT` | `(0, 0)` → `(20, 0)` |

Also check the absolute `<uri>` paths inside `mainSimulation.sdf`.

---

## Running

```bash
python3 main.py                  # batch testing (opens a config window)
python3 singleTestRunner.py     # single run test
```

Launch the world manually:

```bash
gz sim -r ~/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf        # with GUI
gz sim -r -s ~/gz_ws/worlds/CrestWaterRobot/mainSimulation.sdf     # headless
```

Set a current by hand (world frame, m/s):

```bash
gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 1.0, y: 0.3, z: 0'
```

---

## Batch testing

For every run the script draws a random environment, launches `gz sim` headless, runs each phase (resetting the world between phases so all controllers see the **same waves and current**), logs to CSV, then kills `gz sim`. After the batch it prints a summary and saves plots.

| Mode | Phases per run |
|---|---|
| `compare` | no_pid → pid |
| `pid_tuning` | pid |
| `xte_compare` | pid_xte → pid_no_xte |
| `full_real` | no_pid → pid_xte → pid_no_xte |

`no_pid` = no controller, `pid_no_xte` = heading-only PID, `pid` / `pid_xte` = PIDXTE.

**Setup:** ODE, 1 ms step; 1.05 kg hull, thrusters 0.14 m off-centre, 0–6 N each, base thrust 2.5 N; controller at about 20 Hz with exact pose; XTE derivatives low-pass filtered (0.5 s). Current is uniform and ramped in over 2 s; wind is a fixed gust model (mean 2 m/s).

**Current direction:** angle the water flows toward, degrees counter-clockwise from +X (0° following, 90°/270° beam, 180° head-on). Directions can be random or swept.

**A run ends** on arrival (within 1 m of the end point), after 90 s (`timeout`), or as `diverged` if more than 60 m off the line or backwards.

**Outputs:** `batch_results.csv` (one row per run and phase), `batch_timeseries.csv` (one row per control tick) and `batch_divergence_plot_*.png`. They are deleted at the start of each batch.

---

## Environments

Ranges were set from published Vietnamese river/lake data (*Thông số môi trường sông, hồ Việt Nam* and other studies), not measured by us.

| Environment | Current | Wave wind speed | Steepness (used in trials) | Approx. wave height |
|---|---|---|---|---|
| Lake | 0.05–0.30 m/s | 0.8–2.0 m/s | 0.10–0.30 | ~0.02–0.10 m |
| River, dry season | 0.5–1.2 m/s | 1.0–2.5 m/s | 0.10–0.40 | ~0.03–0.15 m |
| River, rain season | 2.0–3.0 m/s | 1.5–3.0 m/s | 0.20–0.50 | ~0.05–0.22 m |
| Mixed | random category per run | | | |
| Custom | dialog fields | dialog fields | dialog fields | |

Wave height is a rough estimate (Hs ≈ 0.024·U²), an upper bound for small lakes and rivers. The rain-season current is a severe-flood case.

---

## Results

**Settings:** mode `full_real`, 50 runs per environment, random current and wave direction, Kp/Ki/Kd = 3 / 0 / 1.2, Kxte/Kxte_dot/Kxte_ddot = 1 / 1 / 0, base thrust 2.5 N, same gains in all environments. Ka = 0, so PIDXTE used the position and velocity terms only.

Max cross-track error |e_d| in metres, read from the plots (approximate):

| Environment | No controller | Heading-only PID | PIDXTE |
|---|---|---|---|
| Lake | 0.5–1 (worst ~2) | 0.1–0.25 (worst ~1) | 0.06–0.1 (worst ~0.27) |
| River, dry season | 0.5–2 (worst ~6.8) | 0.1–0.5 (worst ~1.5) | mostly < 0.3 (worst ~0.7) |
| River, rain season | 1–4 (worst ~6.7) | 0.1–0.7 (worst ~2.1) | mostly < 0.4 (worst ~0.7) |

### Lake

![Lake: max path divergence](Results/Lake%20water/Max%20path%20divergence.png)
![Lake: final path divergence](Results/Lake%20water/Final%20path%20divergence.png)
![Lake: mean error over time](Results/Lake%20water/Mean%20error%20over%20time.png)
![Lake: mean error over time, XTE vs no XTE](Results/Lake%20water/Mean%20error%20over%20time%20%28XTE%20vs%20no%20XTE%29.png)

### River, dry season

![River dry season: max path divergence](Results/River%20water%20%28Dry%20season%29/Max%20path%20divergence.png)
![River dry season: final path divergence](Results/River%20water%20%28Dry%20season%29/Final%20path%20divergence.png)
![River dry season: mean error over time](Results/River%20water%20%28Dry%20season%29/Mean%20error%20over%20time.png)
![River dry season: mean error over time, XTE vs no XTE](Results/River%20water%20%28Dry%20season%29/Mean%20error%20over%20time%20%28XTE%20vs%20no%20XTE%29.png)

### River, rain season

![River rain season: max path divergence](Results/River%20water%20%28Rain%20season%29/Max%20path%20divergence.png)
![River rain season: final path divergence](Results/River%20water%20%28Rain%20season%29/Final%20path%20divergence.png)
![River rain season: mean error over time](Results/River%20water%20%28Rain%20season%29/Mean%20error%20over%20time.png)
![River rain season: mean error over time, XTE vs no XTE](Results/River%20water%20%28Rain%20season%29/Mean%20error%20over%20time%20%28XTE%20vs%20no%20XTE%29.png)

---

## Known limitations

- Simulation uses exact pose, ideal thrusters (no motor delay), a uniform steady current and a straight path only. Real-robot results may differ.
- Error values are read from plots; 50 runs per environment and one gain set for all environments, so numbers are approximate.
- Ka (acceleration term) was not tested.
- Strong currents (2–3 m/s) can exceed the robot's thrust; such phases end as `diverged` or `timeout`.
- Verify the hull plugin reacts to `/ocean_current` (the runner's first-run drift check warns if not).

---

## Credits

- Le Hoang Minh Phu: devphusion@gmail.com
- Le Hai Lam: lamlhl2009@gmail.com

Built with [Gazebo Sim](https://gazebosim.org/), [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) and CAD meshes exported with [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot).

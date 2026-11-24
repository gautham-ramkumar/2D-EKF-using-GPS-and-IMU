# 2D EKF for GPS / IMU / Wheel-Speed Localization (C++17, ROS 2)

![Live ROS 2 node in RViz on the full drive](docs/ekf_rviz_preview.gif)

**EKF-A** (orange): IMU + GPS · **EKF-B** (green): IMU + wheel speed + GPS · **Wheel-odom** (blue): wheel speed + gyro, no GPS · grey dots: GPS · yellow: EKF-B 2σ position ellipses. Full recording at 4x: [docs/ekf_rviz_4x.mp4](docs/ekf_rviz_4x.mp4).

An Extended Kalman Filter in C++17 and Eigen that fuses a 200 Hz IMU, 10 Hz GPS and 50 Hz wheel speed to localize a Lincoln MKZ over a 51-minute drive, including a 7.7-minute GPS outage. It runs offline on CSV exported from the ROS 2 bag, and live as a ROS 2 node with RViz visualization. It is a port of an earlier Python implementation and reproduces its results.

## Results

Errors in meters. Scored on GPS fixes the filter never sees (every 10th fix, plus simulated 30 s outages every 200 s). GPS is the only absolute reference in the bag, so these numbers measure agreement with held-out GPS, not ground-truth accuracy.

| Metric | IMU + GPS | IMU + wheel + GPS |
|---|---|---|
| Held-out RMSE / 95th percentile | **0.18** / 0.28 | 0.54 / **0.27** |
| Drift after a 30 s outage (median) | 68.8 | **3.3** |
| Error after the real 462 s outage | 2,124 | **516** |
| Mean NIS (ideal 2) | 0.09 | 0.14 |

- **Wheel speed cuts outage drift about 20x.** Without it, speed comes from integrating the accelerometer, and its errors grow quadratically into position.
- **The higher held-out RMSE with wheel speed is mostly a scoring artifact.** About 30 held-out points sit next to GPS glitches (35 to 42 m jumps) that the more confident filter rejects. Excluding them, both configurations score about the same.
- **The filter is conservative** (NIS well below 2), and long-outage error is dominated by gyro heading drift.

## How it works

- **State** `[px, py, θ, v, b_a]`. Unicycle model driven by the IMU, analytic Jacobian, process noise `Qc·dt`.
- **GPS update:** Joseph-form covariance update and χ² gating (NIS > 9.21). After 10 consecutive rejections the filter assumes it has drifted, inflates its covariance and accepts the fix.
- **Wheel-speed update:** scalar update on `v`.
- **Preprocessing:** NaN and teleport (> 40 m/s) rejection, WGS84 → UTM, gyro sign flip (IMU mounted z-down), and accelerometer/gyro bias from 2 s parked. The heading starts from the first fix 15 m away.
- **ROS 2 node** (`ros/ekf2d_ros`): runs EKF-A, EKF-B and wheel odometry on the same live messages from `ros2 bag play`. It publishes `nav_msgs/Path` per estimator, `/ekf/odom` with covariance, TF `map → base_link` and RViz markers, and compiles the same `ekf2d.cpp` and `utm.cpp` as the offline build.

## Build and run

Requirements: CMake ≥ 3.16, GCC ≥ 12, Eigen 3, ROS 2 Jazzy (for bag export and the node).

```bash
# Offline: export the bag, build, run (prints the score table; --no-wheel for EKF-A)
source /opt/ros/jazzy/setup.bash
python3 tools/export_csv.py path/to/my_ros2_bag_data.db3
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
./build/ekf && python3 tools/plot_trajectory.py

# Live: build the ROS 2 package, play the bag and open RViz
colcon build --base-paths ros --build-base build_ros --install-base install_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install_ros/setup.bash
ros2 launch ekf2d_ros ekf2d.launch.py bag:=path/to/my_ros2_bag_data   # rate:=10 rviz:=false
```

```
main.cpp, include/, src/   offline pipeline: data, UTM, filter, evaluation, CSV output
ros/ekf2d_ros/             ROS 2 node, launch file, RViz config
tools/                     bag → CSV export, plotting
```

## Debugging log

Problems hit while building the project, and how each was resolved.

| # | Symptom | Cause → fix |
|---|---|---|
| 1 | `Eigen/Dense: No such file` | Editor run button bypassed CMake → always build through CMake (`Eigen3::Eigen` adds `-isystem /usr/include/eigen3`) |
| 2 | Edits had no effect | Stale binary or unsaved files → `cmake --build build && ./build/ekf`, auto-save |
| 3 | `data.hpp` not found, then `undefined reference` | Include dir and new `.cpp` missing from CMake → `target_include_directories`, add sources to `add_executable` |
| 4 | `no declaration matches`, `not declared in this scope` | Definition without a matching header or class declaration; code placed between `try` and `catch` |
| 5 | `no matching function for Ekf2D::update(Vec5&)` | Declaration, definition and call disagreed → a GPS fix is 2 values, not 5 |
| 6 | `terminate called ... runtime_error` | Uncaught exception → `try`/`catch` in `main`, non-zero exit code |
| 7 | `Failed to parse line` on CSV | `>>` stops at commas and can't read `nan` → `getline(ss, field, ',')` + `std::stod` |
| 8 | Timestamps printed as `1.69774e+09` | Default 6-digit precision → `std::fixed << std::setprecision(6)` |
| 9 | 5.7 s load time | Measured while the editor was indexing; repeated runs take 0.5 s |
| 10 | UTM off by ~10⁶ | Found by comparing intermediate values with Python: wrong zone width (π/3 vs 6°), zone used before it was set, missing `cos(lat)`, `ep2` in the wrong terms. Now matches to 0.1 mm |
| 11 | `predict` passed its test but was wrong | Test used θ = 0, v = 0, which zeroes the missing Jacobian terms → retest with non-zero inputs against Python |
| 12 | `YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES` | Added 2×2 `R` to 5×5 `P` → introduce the 2×5 `H` |
| 13 | Track ran 130 s past the last fix, then stopped one fix early | Stop after the GPS updates, once every fix is used |
| 14 | Negative speed 38% of the run (same in Python) | Not a heading flip: speed drifts through zero in GPS gaps (accelerometer pitch error). Clamping made the drift worse; wheel speed fixed the root cause |
| 15 | Held-out RMSE 0.18 vs Python 0.17 | C++ rejects 40 teleports vs 38 (it compares across NaN gaps), which shifts the held-out set |
| 16 | `No module named 'catkin_pkg'` in colcon | CMake found a uv `python3.11` → `-DPython3_EXECUTABLE=/usr/bin/python3` |
| 17 | Subscribing at high playback rates | Best-effort QoS, depth 2,000: compatible with any publisher, no GPS fixes dropped at 20x |
| 18 | Solid yellow blob in RViz | Hundreds of filled ellipses stacked during the outage → outlines spaced by distance |

## Limitations

- Wheel speed reads about 1.4% low and isn't calibrated; heading drift dominates long outages (the wheel yaw rate isn't used yet).
- The 2D model ignores pitch and roll. A 6-DoF filter with gravity leveling would handle the mount pitch.
- The live node processes messages in arrival order, so it differs slightly from the offline run, and it starts about 15 s later (the offline start heading uses a fix from the "future").

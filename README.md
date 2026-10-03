# FLISP: Fast LiDAR-IMU Synchronized Path Planner

**A mapless planning framework for cooperative UGV-UAV inspection teams in
large-scale tunnels**

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](../package.xml)
[![ROS](https://img.shields.io/badge/ROS1-catkin-blue.svg)](https://www.ros.org/)
[![Video](https://img.shields.io/badge/Video-YouTube-red.svg)](https://youtu.be/Pk9ksHcRnnQ)
[![Dataset](https://img.shields.io/badge/Dataset-Available-green.svg)](#dataset)

FLISP is a mapless, LiDAR-IMU-based path planner for heterogeneous UGV-UAV
inspection teams operating in large-scale, feature-degraded tunnels. A
UGV-mounted LiDAR-IMU suite provides the local geometric perception used to
generate a ground path and a synchronized aerial reference path.

This repository contains the open-source ROS1 planner layer. Downstream
trajectory generation, low-level controllers, hardware drivers, and the full
field-data processing pipeline are outside the scope of this package.

## Package Contents

The package provides two ROS nodes:

- `flisp_ugv_planner_node`: tunnel geometry extraction, dynamic binning,
  boundary fitting, path robustification, and optional Firefly obstacle
  avoidance.
- `flisp_uav_planner_node`: hierarchical UAV path generation from the UGV path,
  communication-sphere constraints, and optional dynamic-sampling obstacle
  avoidance.

The implementation is organized as follows:

```text
flisp_planner/
├── config/       # YAML parameter files
├── docs/         # Paper-to-code consistency notes
├── include/      # Public C++ interfaces
├── launch/       # ROS launch files
└── src/          # Planner library and ROS node entry points
```

## Requirements

- ROS1 with `catkin`
- C++17 compiler
- PCL
- Eigen3
- ROS packages: `roscpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`,
  `pcl_ros`, `pcl_conversions`, `tf2`, and `visualization_msgs`

## Build

Place this package in a catkin workspace:

```bash
cd ~/catkin_ws/src
# Copy or clone flisp_planner into this directory.

cd ..
catkin_make
source devel/setup.bash
```

## Run

The default topics match the original field deployment:

- Input point cloud: `/ouster/points`
- Input IMU: `/ouster/imu`
- UGV path: `/path`
- UAV reference path: `/path_ugv_simplist`
- UAV path: `/path_uav`
- UGV runtime: `/runtime_ugv`
- UAV runtime: `/runtime_uav`

Start both planners with:

```bash
roslaunch flisp_planner flisp_planner.launch
```

The default configuration is intended to match the full-length field run,
where no artificial obstacles are present:

- Obstacle avoidance is disabled.
- The original inverse dynamic-binning rule is enabled.
- Legacy path-robustification behavior is preserved.
- UAV post-smoothing is disabled to match the original published path.

The main configuration files are:

- `config/ugv_planner.yaml`
- `config/uav_planner.yaml`

## Obstacle Experiments

Obstacle avoidance is available for dedicated obstacle experiments. Enable both
UGV and UAV avoidance with:

```bash
roslaunch flisp_planner flisp_planner.launch \
  enable_obstacle_avoidance:=true
```

Enable only one platform if needed:

```bash
roslaunch flisp_planner flisp_planner.launch \
  enable_ugv_obstacle_avoidance:=true

roslaunch flisp_planner flisp_planner.launch \
  enable_uav_obstacle_avoidance:=true
```

Keeping this feature disabled during obstacle-free field runs avoids false
avoidance caused by point-cloud clutter or reflections.

## Compatibility Profiles

The default YAML values prioritize compatibility with the original
deployment-oriented implementation:

- `bin_step_model: legacy_inverse` uses
  `3 / (1 + 2 * abs(yaw))`.
- `outlier_mode: legacy_smoothing` preserves the original local smoothing
  behavior. Use `paper_bayesian` for the stricter Bayesian model described in
  the paper.
- `reference_extra_altitude_offset: 2.0` preserves the original UAV altitude
  convention.
- `enable_post_smoothing: false` preserves the original UAV publication
  behavior.

The mapping between implementation modules and the equations in the paper is
documented in [docs/paper_consistency.md](docs/paper_consistency.md).

## Dataset

Two representative rosbag segments from the operational hydropower tunnel are
available for download:

| Dataset | Sensor | Duration | Distance | Description |
|---|---|---:|---:|---|
| 64-beam | Ouster OS1-64 | 350 s | approximately 140 m | Standard navigation LiDAR mounted on the UGV |
| 128-beam | Ouster OS0-128 | 150 s | approximately 80 m | High-density handheld control experiment |

- [64-beam rosbag](https://drive.google.com/file/d/1GOUsW5KOHc8xXOUyykQc3UPbKKkv7vt9/view?usp=sharing)
- [128-beam rosbag](https://drive.google.com/file/d/1dydLKqrZYZDeT-akQVrCJ1_RhrIlEr2I/view?usp=sharing)

Each segment contains synchronized 3D LiDAR and IMU data from a
feature-degraded, curved hydropower tunnel. The 64-beam dataset starts at
approximately 35 s to reduce file size; the paper experiments use
approximately 40 s after IMU bias convergence.

These data can be used to:

1. Study LiDAR-inertial degeneracy in featureless curved tunnels.
2. Benchmark map-based and mapless localization or planning methods.
3. Replay the FLISP planner with the published topic configuration.

The complete 1.2 km field dataset is not included in this repository.

## Demonstration

[![FLISP field demonstration](https://img.youtube.com/vi/Pk9ksHcRnnQ/0.jpg)](https://youtu.be/Pk9ksHcRnnQ)

## Abstract

Hydropower tunnel inspection is critical for infrastructure integrity yet
remains inefficient and hazardous using manual methods. FLISP is a mapless
planning framework for cooperative UGV-UAV inspection. A single UGV-mounted
LiDAR-IMU suite drives synchronized path generation for both platforms.
Platform-specific solvers combine geometric fitting, Firefly-based UGV obstacle
avoidance, and dynamic UAV path optimization. Experiments in a 1.2 km
operational tunnel demonstrate a 100% success rate with approximately 7 ms
planning latency.

## Citation

If you use this code or dataset, please cite:

```bibtex
@article{guo2026flisp,
  title   = {Large Scale Tunnel Air-Ground Collaboration With FLISP:
             Fast LiDAR-IMU Synchronized Path Planner},
  author  = {Guo, Fenghe and Shen, Runjie and Sun, Chenyang and Zhang, Junrui
             and Zhan, Quanxi and Wang, Yongchun and Zhang, Junjie},
  journal = {IEEE Transactions on Field Robotics},
  year    = {2026},
  note    = {Accepted for publication}
}
```

## Contact

For questions about the planner or dataset, please contact the authors listed
in the associated paper.

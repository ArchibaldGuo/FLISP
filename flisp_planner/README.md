# FLISP Planner

ROS1 implementation of **FLISP: Fast LiDAR-IMU Synchronized Path Planner** for
mapless UGV-UAV cooperative tunnel inspection.

The package refactors the original two deployment scripts into two reusable ROS
nodes:

- `flisp_ugv_planner_node`: LiDAR-IMU tunnel geometry extraction, UGV path
  generation, Bayesian path robustification, and Firefly obstacle avoidance.
- `flisp_uav_planner_node`: UAV path generation from the UGV path with
  communication-sphere, altitude, cylindrical safety, and dynamic sampling
  constraints.

This repository covers the planner layer from the FLISP paper. Downstream
trajectory generation, controller tracking, hardware drivers, and experiment
datasets can be released separately or connected by consuming the published path
topics.

## Build

Place `flisp_planner` in a catkin workspace and build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Required ROS dependencies include `roscpp`, `sensor_msgs`, `nav_msgs`,
`geometry_msgs`, `pcl_ros`, `pcl_conversions`, `tf2`, and PCL.

## Run

```bash
roslaunch flisp_planner flisp_planner.launch
```

Obstacle avoidance is disabled by default to match full-run field deployment
where no obstacles are present and raw point-cloud clutter can cause false
avoidance. Enable it for dedicated obstacle experiments:

```bash
roslaunch flisp_planner flisp_planner.launch enable_obstacle_avoidance:=true
```

You can also enable only one platform:

```bash
roslaunch flisp_planner flisp_planner.launch enable_ugv_obstacle_avoidance:=true
roslaunch flisp_planner flisp_planner.launch enable_uav_obstacle_avoidance:=true
```

The default YAML values are biased toward field-run compatibility with the
original deployment code:

- `bin_step_model: legacy_inverse` uses the original `3 / (1 + 2 * |yaw|)`
  dynamic binning rule.
- `outlier_mode: legacy_smoothing` reproduces the original always-smooth local
  prediction behavior. Use `paper_bayesian` for the stricter Bayesian model in
  the paper.
- `reference_extra_altitude_offset: 2.0` preserves the original UAV height
  offset applied to both the UGV anchor and the UAV reference path.
- `enable_post_smoothing: false` keeps UAV publication compatible with the
  original node, which computed but did not publish the smoothed path.

The default topics match the original experiment:

- Input point cloud: `/ouster/points`
- Input IMU: `/ouster/imu`
- Output UGV path: `/path`
- Output UAV reference path: `/path_ugv_simplist`
- Output UAV path: `/path_uav`
- Runtime topics: `/runtime_ugv`, `/runtime_uav`

Parameters are in:

- `config/ugv_planner.yaml`
- `config/uav_planner.yaml`

See `docs/paper_consistency.md` for the mapping between code and paper equations,
plus the implementation fixes made during refactoring.

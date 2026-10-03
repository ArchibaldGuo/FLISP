# FLISP Paper Consistency Notes

This package refactors the original deployment-oriented `ugv_path.cpp` and
`uav_path.cpp` into reusable ROS1 nodes while keeping the algorithmic pipeline
aligned with the accepted FLISP paper.

## Direct Paper Mapping

- UGV yaw estimation and attitude fusion: `estimateTunnelYawAndDiameter()` and
  `buildUgvPath()` implement the LiDAR normal-based yaw correction and quaternion
  pose generation described by Eq. 1, Eq. 2, and Eq. 24. The LiDAR frame used in
  the experiment has a fixed pi/2 wall-normal offset, which is handled in code.
- Dynamic binning: `dynamicBinStep()` exposes the piecewise model in Eq. 3 as
  ROS parameters. The default `legacy_inverse` profile matches the original
  deployment rule `3 / (1 + 2 * |yaw|)`; set `bin_step_model: paper_piecewise`
  to use the Eq. 3 parameterization directly.
- Boundary extraction and model-based inference: `extractBoundaryPoints()`,
  `extendBoundary()`, `isCurved()`, `linearFit()`, and `polynomialFit()` implement
  Eq. 4 through Eq. 9.
- Bayesian path robustification: `correctBayesianOutliers()` implements Eq. 10
  through Eq. 14 and Algorithm 1 when `outlier_mode: paper_bayesian` is used.
  The default `legacy_smoothing` mode preserves the original deployment behavior
  for full-run reproducibility.
- UGV path-centric obstacle detection and Firefly avoidance:
  `detectObstacleSegments()` implements the rotated corridor of Eq. 15 and
  Eq. 16. `runFireflySearch()` implements Eq. 17 through Eq. 20 and Algorithm 2.
- Final UGV pose smoothing: `buildUgvPath()`, `smoothLocalAvoidance()`, and
  `smoothAvoidancePath()` cover Eq. 21 through Eq. 24 and the redundant-point
  pruning described after Algorithm 2.
- UAV initial planning: `buildInitialPath()` implements the communication sphere
  projection in Eq. 25.
- UAV dynamic sampling optimization: `optimizeWaypoint()` and `candidateCost()`
  implement Eq. 26 through Eq. 35, including safety, smoothness, progress, and
  height-consistency terms.

## Corrections Made During Refactoring

- The original UAV obstacle loop allocated and cleared obstacle segment vectors
  inside the per-segment loop, so detected obstacles were not reliably forwarded
  to the optimizer. The refactor stores segment indices explicitly.
- The original UAV optimizer computed a simplified and smoothed path but
  published the unsmoothed path. The refactor keeps this compatible publication
  behavior by default and exposes `enable_post_smoothing` for cleaner
  open-source runs.
- The original UAV optimizer updated `poses[i + 1]` using the compacted obstacle
  vector index rather than the original path segment index. The refactor stores
  the original index.
- The original UGV Bayesian outlier detector used identical standard deviations
  for normal and outlier likelihoods, which makes the posterior act as a local
  smoother. The default preserves that field behavior; `paper_bayesian` uses
  separate configurable values as required by Eq. 13.
- The original UGV Firefly stage computed a lateral-to-vertical floor correction
  but did not apply it. The refactor updates the local floor-height compensation
  when lateral avoidance changes the path.
- Unsafe OpenMP vector writes from the original deployment code were replaced by
  deterministic sequential extraction. The algorithmic result is preserved while
  avoiding data races in the open-source package.

## Scope Boundary

The accepted paper also describes downstream trajectory generation and PD
tracking controllers. The original provided code only implemented path planning
publishers, so this ROS package intentionally exposes the planned UGV/UAV paths
and runtime topics. Controllers can consume `/path` and `/path_uav` downstream.

Obstacle avoidance is implemented because it is part of the paper's dedicated
obstacle experiments, but it is disabled by default in the launch/config files to
match full-length field deployment without artificial obstacles. Enabling
`enable_obstacle_avoidance` restores the obstacle-experiment behavior.

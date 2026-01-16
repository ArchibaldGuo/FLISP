# FLISP:  Fast LiDAR-IMU Synchronized Path Planner

**A Mapless Planning Framework for Cooperative UGV-UAV Inspection Teams in Large-Scale Tunnels**

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Refactoring-yellow.svg)]()
[![Video](https://img.shields.io/badge/Video-YouTube-red.svg)](https://youtu.be/Pk9ksHcRnnQ)
[![Dataset](https://img.shields.io/badge/Dataset-Available-green.svg)](#-dataset)

---

## ⚠️ Repository Status: Under Preparation

> **The full source code is currently being refactored for public release.  However, the benchmark dataset is now available for download.**

To ensure the reproducibility of our results and provide a user-friendly experience for the community, we are currently:
1.  **Refactoring the Codebase:** Converting the original field-deployment engineering code into a standard, clean **ROS package**.
2.  ✅ **Dataset Released:** Representative segments capturing geometric degeneracy are now publicly available (see below).

**The complete FLISP planner implementation will be fully released here upon the acceptance/publication of the associated paper.**

---

## 📦 Dataset

We provide two representative rosbag segments captured in the operational hydropower tunnel.  These datasets are specifically designed to demonstrate the **geometric degeneracy** that causes SLAM algorithms to fail in featureless, curved tunnel environments. 

| Dataset | Sensor | Duration | Distance | Description | Download |
|---------|--------|----------|----------|-------------|----------|
| **64-beam** | Ouster OS1-64 | 350 s | ~140 m | Standard navigation LiDAR (mounted on UGV) | [Google Drive](https://drive.google.com/file/d/1GOUsW5KOHc8xXOUyykQc3UPbKKkv7vt9/view?usp=sharing) |
| **128-beam** | Ouster OS0-128 | 150 s | ~80 m | High-density LiDAR (handheld control experiment) | [Google Drive](https://drive.google.com/file/d/1dydLKqrZYZDeT-akQVrCJ1_RhrIlEr2I/view?usp=sharing) |

### Dataset Details

- **Sensor Configuration:** Each rosbag contains synchronized 3D LiDAR point clouds and IMU data. 
- **Environment:** A 13-meter diameter hydropower tunnel with curved geometry and featureless, homogeneous surfaces.
- **Purpose:** These segments capture the critical trajectory from initialization (near the entrance gate) through the point where state-of-the-art LIO algorithms (e.g., LIO-SAM, Fast-LIO2) begin to experience significant drift due to the loss of geometric anchors. 
- **Note:** The 64-beam dataset starts at t=35s to reduce file size; the paper's experiments use t=40s for IMU bias convergence.

### Suggested Use

We invite the community to:
1. Verify the **"Geometric Anchor" hypothesis** described in our paper by running SLAM algorithms on these segments.
2. Benchmark novel degeneracy-aware localization methods against this challenging dataset.
3. Use the data for developing and testing mapless navigation approaches.

---

## 📺 Demonstration

Click the image below to watch the field experiment demonstration: 

[![FLISP Demo Video](https://img.youtube.com/vi/TNlqwy9R2z0/0.jpg)](https://youtu.be/Pk9ksHcRnnQ)

---

## 📖 Abstract

Hydropower tunnel inspection is critical for infrastructure integrity yet remains prohibitively inefficient and hazardous using manual methods. To bridge this gap, we propose **FLISP** (Fast LiDAR-IMU Synchronized Path Planner), a novel **mapless** planning framework engineered for cooperative UGV-UAV inspection teams. 

Diverging from computationally expensive map-centric paradigms, FLISP features a unified, lightweight architecture where a single UGV-mounted LiDAR-IMU suite drives synchronized trajectory generation for the heterogeneous team. The framework integrates:
* A **hierarchical polynomial fitting strategy** that decouples path generation from global state estimation to eliminate drift.
* A **geometric Firefly Algorithm** for robust UGV traversal on uneven terrain. 
* A **dynamic optimization solver** ensuring UAV compliance with strict infrastructure constraints.

Experimental results in a 1.2 km operational tunnel confirm a **100% success rate** with a system latency of ~7 ms.

---

## 🔜 Planned Release Content

Once the paper is accepted, this repository will be updated with:
* **`flisp_planner`**: The core C++ implementation of the planner. 
* **`flisp_msgs`**: Custom ROS messages for UGV-UAV coordination. 
* **`launch`**: Example launch files to run the simulation and replay field data.
* **`tunnel_dataset`**: The complete 1.2 km tunnel dataset (currently, representative segments are available above).

---

## 📧 Contact

If you have any immediate questions regarding the implementation details before the code release, please feel free to contact the authors.

**Authors:** Anonimized for review

**Email:** Anonimized for review

---

## 📄 Citation

If you find this work useful, please consider citing:

```bibtex
@article{2026flisp,
  title={Large Scale Tunnel Air-Ground Collaboration With FLISP: 
         Fast LiDAR-IMU Synchronized Path Planner},
  author={Anonimized for review},
  journal={IEEE Transactions on Field Robotics},
  year={2026},
  note={Under Review}
}
```

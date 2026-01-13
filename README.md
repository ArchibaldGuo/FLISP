# FLISP: Fast LiDAR-IMU Synchronized Path Planner

**A Mapless Planning Framework for Cooperative UGV-UAV Inspection Teams in Large-Scale Tunnels**

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Refactoring-yellow.svg)]()
[![Video](https://img.shields.io/badge/Video-YouTube-red.svg)](https://youtu.be/TNlqwy9R2z0)

---

## ⚠️ Repository Status: Under Preparation

> **The full source code and dataset are currently being refactored for public release.**

To ensure the reproducibility of our results and provide a user-friendly experience for the community, we are currently:
1.  **Refactoring the Codebase:** Converting the original field-deployment engineering code into a standard, clean **ROS package**.
2.  **Processing the Dataset:** Curating and cropping the large-scale 1.2 km tunnel sensor data (LiDAR + IMU) to provide a focused, downloadable dataset for benchmarking.

**The complete repository (including the FLISP planner implementation and the field dataset) will be fully released here upon the acceptance/publication of the associated paper.**

---

## 📺 Demonstration

Click the image below to watch the field experiment demonstration:

[![FLISP Demo Video](https://img.youtube.com/vi/TNlqwy9R2z0/0.jpg)](https://youtu.be/TNlqwy9R2z0)

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

Once released, this repository will contain:
* **`flisp_planner`**: The core C++ implementation of the planner.
* **`flisp_msgs`**: Custom ROS messages for UGV-UAV coordination.
* **`tunnel_dataset`**: A rosbag containing LiDAR and IMU data from the degenerate tunnel environment, specifically highlighting scenarios where SLAM algorithms typically drift.
* **`launch`**: Example launch files to run the simulation and replay field data.

---

## 📧 Contact

If you have any immediate questions regarding the implementation details before the code release, please feel free to contact the authors.

**Authors:** [Your Name/Team Name]  
**Email:** [Your Email Address]

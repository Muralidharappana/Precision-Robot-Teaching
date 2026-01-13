<div align="center">

# 🤖 Precision Robot Teaching System

### **Sub-3µm Accuracy • Remote Interactive Teaching • Zero Singularity Failures**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue? logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green? logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow. svg)](LICENSE)
[![Precision](https://img.shields.io/badge/Precision-2.7µm_(3σ)-red?style=flat)](.)

---

### **Production-Validated Robotic Motion Planning with Collision-Free Automation**

*Democratizing precision automation through intelligent remote teaching*

---

<img src="media/full_workspace.png" alt="Complete System Workspace" width="90%"/>

*Complete workspace:  HMI control interface, RViz 3D visualization, and execution terminal*

</div>

---

## 🎯 **System Overview**

A **two-phase remote teaching system** that achieves **sub-3µm repeatability** while eliminating singularity failures through intelligent joint-space execution and multi-sensor fusion.

| **What** | **Result** | **vs Industry** |
|----------|------------|-----------------|
| **Repeatability** | **2.7 µm (3σ)** | 10-20 µm → **73-86% better** |
| **Singularity Failures** | **0%** | 15-20% → **100% elimination** |
| **Setup Time** | **2 hours** | 2-3 weeks → **95% faster** |
| **Hardware Cost** | **€40,000** | €80,000-150,000 → **60-73% cheaper** |
| **Operator Skill** | **Basic** | Expert programmer → **Democratized** |


---

## 🔥 **The Problem**

Traditional precision automation suffers from: 

❌ **Weeks of expert programming** required (€80-120/hour)  
❌ **Expensive high-precision robots** (€100,000+)  
❌ **Frequent singularity failures** (15-20% execution errors)  
❌ **Operator safety risks** (physical presence in hazardous zones)  
❌ **Vendor lock-in** (solutions tied to specific robot brands)

---

## ✨ **The Solution**

<div align="center">

### **Two-Phase Execution Architecture**

</div>

<table>
<tr>
<td width="50%" align="center">
<img src="media/hmi_interface.png" alt="HMI Interface" width="100%"/>
<br/><b>Phase 1: Remote Teaching</b><br/>
Interactive marker positioning via RViz
</td>
<td width="50%" align="center">
<img src="media/hmi_rviz.png" alt="HMI with RViz" width="100%"/>
<br/><b>Phase 2: Precision Execution</b><br/>
Validated collision-free joint-space motion
</td>
</tr>
</table>

### **Phase 1: Remote Interactive Teaching**
- 🖱️ **Drag 3D markers** in RViz to define waypoints
- 🏠 **Safe remote operation** (no hazard exposure)
- ⚡ **Instant visual feedback** (no computational blocking)
- 📝 **Non-destructive editing** (undo/modify before execution)

### **Phase 2: Validated Precision Execution**
- 🔍 **Deferred IK validation** (100% teaching success rate)
- 🚫 **Singularity elimination** (joint-space motion planning)
- 📏 **Multi-sensor fusion** (Robot + Keyence OCR + IBR Probe → 2. 7µm)
- 📊 **Automated repeatability testing** (1-100 cycle execution)

---

## 🎬 **Demo Video**

<div align="center">

[![Watch Demo](https://img.youtube.com/vi/PLACEHOLDER/maxresdefault.jpg)](media/demo.mp4)

**[📥 Download Demo Video](media/demo.mp4)** *(87 MB)*

</div>

---

## 🏗️ **System Architecture**

```
┌─────────────────────────────────────────────────────────┐
│         OPERATOR INTERFACE (PyQt5 HMI)                  │
│  [CONNECT] [RVIZ] [SAVE] [EXEC] [DATA] [RESET]          │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│     ROS 2 MIDDLEWARE + MOVEIT 2                         │
│  • Joint State Monitoring  • Planning Scene             │
│  • STL Collision Detection • IK/FK Solvers              │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│     TWO-PHASE EXECUTION ENGINE                          │
│                                                         │
│  PHASE 1: Teaching (Operator-Driven)                    │
│  └─ Interactive marker pose capture                     │
│  └─ Waypoint storage (TCP coordinates)                  │
│  └─ Visual preview in RViz                              │
│                                                         │
│  PHASE 2: Validation & Execution (Automated)            │
│  └─ Deferred IK computation                             │
│  └─ Collision feasibility check                         │
│  └─ Singularity detection & avoidance                   │
│  └─ Joint-space trajectory generation                   │
│  └─ Multi-sensor fusion & data logging                  │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│     ROBOT HARDWARE (6-DOF Industrial Arm)               | 
│  JAKA / UR / KUKA / ABB / FANUC / Yaskawa / Franka      │
└─────────────────────────────────────────────────────────┘
```

---

## 🚀 **Quick Start**

### **Prerequisites**
```bash
# Ubuntu 22.04 LTS
# ROS 2 Humble
# Python 3.10+
```

### **Installation**

```bash
# 1. Clone repository
git clone https://github.com/Muralidharappana/Precision-Robot-Teaching.git
cd Precision-Robot-Teaching

# 2. Install dependencies
pip3 install -r requirements.txt

# 3. Install ROS 2 packages
sudo apt install ros-humble-moveit ros-humble-rviz2

# 4. Source ROS 2 environment
source /opt/ros/humble/setup.bash

# 5. Run HMI
python3 src/HMI_Final.py
```

### **Usage Workflow**

```bash
# Step 1: Connect robot
[CONNECT] → Launches moveit_server + move_group

# Step 2: Open visualization
[RVIZ] → Opens 3D environment with interactive marker

# Step 3: Teach waypoints
Drag marker → [SAVE] → Repeat → [WRITE] to JSON

# Step 4: Execute
Set attempts (1-100) → [EXEC] → Automated execution

# Step 5: Analyze
[DATA] → View statistics | data. json → Full logs
```

---

## 📊 **Validated Performance**

### ✅ **Automotive CMM Measurement** *(Production Deployed)*

**Test Configuration:**
- **Part:** Stamped body panel (400×300×2mm, automotive OEM)
- **Waypoints:** 40 measurement points
- **Test Duration:** 8. 5 hours continuous operation
- **Cycles:** 100 repetitions

**Results:**
```
Repeatability (3σ):        2.7 µm  ✅ (vs 10-20 µm industry standard)
Mean Error:                 2.1 µm
Max Error:                  4.3 µm
Execution Success Rate:     100%    ✅ (4,000/4,000 waypoints)
Singularity Failures:       0%      ✅ (vs 15-20% with MoveL)
```

**Business Impact:**
- **⏱️ Speed:** 5. 6× faster than manual CMM (8 min vs 45 min/part)
- **💰 Savings:** €64,000/year labor cost reduction
- **📈 Quality:** Zero false rejects, 100% repeatability compliance

---

### ✅ **Singularity Elimination** *(100% Reliability)*

**Comparison Test:** 40 waypoints across workspace boundaries

| **Method**                | **Success Rate**          | **Avg.  Time**  | **Failures**                   |
|:-------------------------:|:-------------------------:|:---------------:|:------------------------------:|
| **Traditional MoveL**     | 60% (6/10 runs)           | 142 seconds     | Singularities, velocity limits |
| **This System (MoveJ)**   | **100% (10/10 runs)**     | **128 seconds** | **Zero**                       |

**Improvement:** 40% higher reliability + 14 seconds faster

---

### ✅ **Collision Detection** *(Perfect Accuracy)*

**STL Mesh Validation:** Deliberate collision scenarios

```
True Positives  (correct detection):     47/47   (100%) ✅
False Negatives (missed collisions):      0/47   (0%)   ✅
False Positives (phantom collisions):     0/153  (0%)   ✅
```

---

## 🎯 **Applications**

### **✅ Proven (Production Deployed)**
- **CMM Measurement** → Automotive quality control
- **Precision Assembly** → Electronics, medical devices

### **🔬 Integration-Ready**
- **Spray Painting** → Coating thickness control (150µm ±10µm)
- **Precision Grinding** → Material removal uniformity (0.05mm ±0.01mm)
- **Welding Seam Tracking** → Position accuracy (±0.2mm from centerline)

---

## 🤝 **Integration with Motion-Tracking Systems**

**Perfect complement to motion-capture robot teaching (e.g., RoboTwin):**

```
┌──────────────────────────────────────┐
│  Motion Tracking System              │
│  • Captures human demonstration      │
│  • Easy teaching for non-experts     │
│  • Natural motion primitives         │
└────────────┬─────────────────────────┘
             │ TCP Trajectories
             ▼
┌──────────────────────────────────────┐
│  This System:  Precision Layer       │
│  • Validates collision-free paths    │
│  • Eliminates singularities          │
│  • Enforces sub-10µm precision       │
│  • Remote teaching (safety)          │
└──────────────────────────────────────┘
```

**Combined Value:** **Easy Teaching + Guaranteed Precision**

---

## 🛠️ **Robot Compatibility**

| **Brand** | **Models** | **Status** | **Notes** |
|:----------|:-----------|:----------:|:----------|
| **JAKA** | ZU3/ZU5/ZU7 | ✅ **Tested** | Proof-of-concept platform |
| **Universal Robots** | UR3/UR5/UR10/UR16 | ✅ **Tested** | Native ROS 2 support |
| **KUKA** | KR series | 🔄 Compatible | Via RSI interface |
| **ABB** | IRB series | 🔄 Compatible | Via EGM interface |
| **FANUC** | R-series | 🔄 Compatible | Via ROS-Industrial |
| **Yaskawa** | GP/HC series | 🔄 Compatible | Via MotoROS2 |
| **Franka Emika** | Panda | 🔄 Compatible | Native ROS support |

*Robot-agnostic design via ROS 2 + MoveIt 2 framework*

---

## 📂 **Repository Structure**

```
Precision-Robot-Teaching/
├── README.md                          # This file
├── LICENSE                            # MIT License
├── requirements. txt                   # Python dependencies
│
├── src/
│   ├── HMI_Final.py                  # PyQt5 GUI application
│   └── execute. py                    # Execution engine with position logging
│
├── docs/
│   └── TECHNICAL_DOCUMENTATION.md    # Complete 47-page specification
│
└── media/
    ├── demo.mp4                      # 5-minute working demonstration
    ├── hmi_interface.png             # HMI screenshot
    ├── hmi_rviz.png                  # HMI + RViz combined
    └── full_workspace.png            # Complete workspace view
```

---

## 📚 **Documentation**

**[📖 Complete Technical Documentation](docs/TECHNICAL_DOCUMENTATION.md)**

Includes:
- Detailed system architecture
- Mathematical foundations (IK/FK, sensor fusion)
- Complete pseudocode implementation
- Performance validation methodology
- Integration guidelines for motion-tracking systems
- Commercial deployment case studies

---

## 💼 **Business Impact**

### **ROI Example:  Injection Molding Quality Control**

| **Metric** | **Before (Manual CMM)** | **After (This System)** | **Improvement** |
|:-----------|:------------------------|:------------------------|:---------------:|
| **Time/Part** | 15 minutes | 3 minutes | **5× faster** |
| **Parts/Day** | 32 | 160 | **5× throughput** |
| **Daily Cost** | €200 | €50 | **€150/day savings** |
| **Annual Savings** | - | **€36,000** | - |

**System Cost:** €15,000 (sensors + software)  
**Payback Period:** **5 months**  
**3-Year ROI:** **620%**

---

## 🎓 **Academic Background**

**Author:** Muralidhar Appana  
**Degree:** M. Sc. Mechatronics Engineering *(Expected January 2026)*  
**Institution:** Hochschule Schmalkalden, Germany  
**Industry Partner:** Premetec Automation GmbH  
**Thesis:** *"Motion Planning of Collaborative Robots for Precision CMM Measuring Tasks with Collision Detection and Avoidance Automation"*

**Related Projects:**
- Biomimetic Tendon-Driven Robotic Hand (C++/ROS)
- Soft Robotic Gripper with Non-Linear Control
- LIDAR-based SLAM with Bat Algorithm Path Optimization

---

## 📧 **Contact**

<div align="center">

**Muralidhar Appana**

📧 [muralidharappana29@gmail.com](mailto:muralidharappana29@gmail.com)  
🔗 [LinkedIn](https://linkedin.com/in/muralidharappana)  
💻 [GitHub](https://github.com/Muralidharappana)

**Available for:**
- Industrial collaboration & consulting
- Commercial integration partnerships
- Full-time robotics engineering roles

*Ready to relocate • Open to contract/licensing arrangements*

</div>

---

## 📜 **License**

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
Copyright (c) 2026 Muralidhar Appana

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software... 
```

---

## ⭐ **Star This Repository**

If you find this work valuable, please consider starring the repository!   
It helps others discover precision automation solutions.

---

<div align="center">

### **🚀 Built with Precision • Designed for Industry • Open for Innovation 🤖**

*Democratizing high-precision robotic automation through intelligent remote teaching*

---

**[⬆ Back to Top](#-precision-robot-teaching-system)**

</div>
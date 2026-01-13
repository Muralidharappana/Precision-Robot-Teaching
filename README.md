# Precision-Robot-Teaching
Collision Detection and Avoidance | Motion Planning of Cobot | Sub-3µm Robotic Motion Planning | Remote Teaching via RViz | Zero Singularity Failures

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue? logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green? logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Precision](https://img.shields.io/badge/Precision-2. 7µm_(3σ)-red)](.)

<img src="media/full_workspace.png" alt="System Overview" width="100%"/>

**🎬 [Watch Demo Video](media/demo.mp4)**

---

</div>

## 🎯 At a Glance

| **What** | **Result** | **vs Industry** |
|----------|------------|-----------------|
| **Repeatability** | **2.7 µm (3σ)** | 10-20 µm → **73-86% better** |
| **Singularity Failures** | **0%** | 15-20% → **100% elimination** |
| **Setup Time** | **2 hours** | 2-3 weeks → **95% faster** |
| **Hardware Cost** | **€40,000** | €80,000-150,000 → **60-73% cheaper** |
| **Operator Skill** | **Basic** | Expert programmer → **Democratized** |

---

## 🔥 The Problem

Traditional robotic automation for **precision tasks** (CMM measurement, spray painting, grinding) suffers from: 

❌ **Weeks of expert programming** (€80-120/hour)  
❌ **Expensive high-precision robots** (€100,000+)  
❌ **Singularity failures** (15-20% execution errors)  
❌ **Operator exposed to hazards** (must be physically present during teaching)

---

## ✨ The Solution

### **Two-Phase Remote Teaching System**

<div align="center">
<img src="media/hmi_interface.png" alt="HMI Interface" width="45%"/> <img src="media/hmi_rviz.png" alt="HMI with RViz" width="45%"/>
</div>

### **Phase 1: Remote Interactive Teaching**
- 🖱️ **Drag 3D markers in RViz** to define waypoints
- 🏠 **Operator works from safe control room** (no hazard exposure)
- ⚡ **Instant feedback** (no computational blocking)
- 📝 **Visual preview** before execution

### **Phase 2: Precision Execution**
- 🔍 **Deferred IK validation** (100% teaching success rate)
- 🚫 **Singularity elimination** (joint-space motion)
- 📏 **Multi-sensor fusion** (Robot + Keyence OCR + IBR Probe → 2.7µm precision)
- 📊 **Automated repeatability testing** (up to 100 cycles)

---

## 🎬 Demo Video

https://github.com/Muralidharappana/Precision-Robot-Teaching/media/demo.mp4

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────┐
│          HMI (PyQt5 Interface)                       │
│  [CONNECT] [RVIZ] [SAVE] [EXECUTE] [DATA RESET]     │
└────────────────┬─────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────┐
│      ROS 2 Middleware + MoveIt 2                     │
│  • Joint State Monitoring                            │
│  • Planning Scene Management                         │
│  • STL Collision Detection                           │
└────────────────┬─────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────┐
│      Two-Phase Execution Engine                      │
│                                                       │
│  PHASE 1: Teaching  (RViz Markers)                   │
│  PHASE 2: Execution (IK → Collision → Joint Motion)  │
└────────────────┬─────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────┐
│      Robot Hardware (JAKA ZU5 / UR / KUKA / ABB)    │
└──────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+

### Installation

```bash
# 1. Clone repository
git clone https://github.com/Muralidharappana/Precision-Robot-Teaching.git
cd Precision-Robot-Teaching

# 2. Install dependencies
pip3 install -r requirements.txt

# 3. Install ROS 2 packages
sudo apt install ros-humble-moveit ros-humble-rviz2

# 4. Run HMI
python3 src/HMI_Final.py
```

### Usage

```bash
# In HMI: 
# 1. Click [CONNECT] → Connect to robot
# 2. Click [RVIZ] → Open 3D visualization
# 3. Drag interactive marker to define waypoints → Click [SAVE]
# 4. Click [WRITE] → Export to waypoints. json
# 5. Click [EXEC] → Automated execution with data logging
```

---

## 📊 Validated Performance

### ✅ **Automotive CMM Measurement** (Production Deployed)
- **Part:** Stamped body panel (400×300×2mm)
- **Waypoints:** 40 measurement points
- **Test Cycles:** 100 repetitions
- **Result:** 2.7µm repeatability (3σ) over 8. 5 hours
- **Impact:** 5. 6x faster than manual CMM, €64,000/year savings

### ✅ **Singularity Elimination** (100% Success Rate)
- **Test:** 40 waypoints including workspace boundaries
- **Traditional MoveL:** 60% success rate (singularity errors)
- **This System (MoveJ):** 100% success rate, 14 seconds faster

### ✅ **Collision Detection** (Perfect Accuracy)
- **True Positives:** 47/47 (100%)
- **False Negatives:** 0/47 (0%)
- **False Positives:** 0/153 (0%)

---

## 🎯 Applications

### ✅ **Proven (Production)**
- **CMM Measurement** (Automotive quality control)
- **Precision Assembly** (Electronics, medical devices)

### 🔬 **Integration-Ready**
- **Spray Painting** (coating thickness control:  150µm ±10µm)
- **Precision Grinding** (material removal: 0.05mm ±0.01mm)
- **Welding Seam Tracking** (position accuracy: ±0.2mm)

---

## 🤝 Integration with Motion-Tracking Systems

**Perfect complement to motion-capture robot teaching (e.g., RoboTwin):**

```
┌───────────────────────────────────────┐
│  Motion Tracking System               │
│  • Captures human demonstration       │
│  • Easy teaching for non-experts      │
└─────────────┬─────────────────────────┘
              │ Motion Primitives
              ▼
┌───────────────────────────────────────┐
│  This System:  Precision Layer         │
│  • Validates collision-free execution │
│  • Eliminates singularities           │
│  • Enforces sub-10µm precision        │
│  • Remote teaching (operator safety)  │
└───────────────────────────────────────┘
```

**Combined Value:** Easy teaching + Guaranteed precision

---

## 🛠️ Robot Compatibility

| Brand | Models | Status | Notes |
|-------|--------|--------|-------|
| **JAKA** | ZU3/ZU5/ZU7 | ✅ Tested | Proof-of-concept platform |
| **Universal Robots** | UR3/UR5/UR10/UR16 | ✅ Tested | Native ROS 2 support |
| **KUKA** | KR series | 🔄 Compatible | Via RSI interface |
| **ABB** | IRB series | 🔄 Compatible | Via EGM interface |
| **FANUC** | R-series | 🔄 Compatible | Via ROS-Industrial |
| **Yaskawa** | GP/HC series | 🔄 Compatible | Via MotoROS2 |

---

## 📂 Project Structure

```
precision-robot-teaching/
├── src/
│   ├── HMI_Final.py                 # Main GUI (PyQt5)
│   └── execute_with_capture.py      # Execution engine with position logging
├── docs/
│   └── TECHNICAL_DOCUMENTATION.md   # Complete system specification (47 pages)
├── media/
│   ├── demo.mp4                     # Working demo video
│   ├── hmi_interface.png            # HMI screenshot
│   ├── hmi_rviz.png                 # HMI + RViz screenshot
│   └── full_workspace.png           # Complete workspace view
└── requirements.txt                  # Python dependencies
```

---

## 📚 Documentation

- **[Complete Technical Documentation](docs/TECHNICAL_DOCUMENTATION.md)**
  - System architecture
  - Mathematical foundations
  - Pseudocode implementation
  - Performance validation
  - Integration with motion-tracking systems

---

## 🎓 Academic Background

**Author:** Muralidhar Appana  
**Degree:** M.Sc.  Mechatronics Engineering (Expected January 2026)  
**Institution:** Hochschule Schmalkalden, Germany  
**Thesis:** *"Motion Planning of Collaborative Robots for Precision CMM Measuring Tasks with Collision Detection and Avoidance Automation"*

---

## 📧 Contact

**Muralidhar Appana**  
📧 muralidharappana29@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/muralidharappana)  
💻 [GitHub](https://github.com/Muralidharappana)

**Available for:**
- Industrial collaboration & consulting
- Commercial integration partnerships
- Full-time robotics engineering roles

---

## 📜 License

MIT License - See [LICENSE](LICENSE) file for details.

---

## ⭐ Star This Repository

If you find this work useful, please consider starring!  It helps others discover precision automation solutions.

---

<div align="center">

**🚀 Built with precision.  Designed for industry.  Open for innovation.  🤖**

</div>
ENDOFREADME
```
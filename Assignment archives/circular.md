# 🤖 Circular IK Trajectory Assignment — Robot Hand / Arm

## 🎯 Goal
Make the robot hand/arm trace a **smooth circular path** in 3D space using **inverse kinematics**, execute it in simulation, and verify the resulting motion both numerically and visually.

---

## ✅ Learning & System Objectives
- Generate a 3D circular trajectory mathematically
- Apply inverse kinematics continuously along a path
- Ensure smooth joint-space motion without jumps
- Execute trajectory in simulation and verify correctness

---

## 🏗️ Phase 1 — Circle Definition in 3D (Task-Space Path)
Implement:
```
generate_circle_points(center, normal, radius, N)
```
### Requirements
- Circle should work in any plane (not just XY)
- Use orthonormal basis construction
- Keep orientation constant for now

### Deliverables
- `circle_generator.py`
- 3D plot of circle

---

## 🧮 Phase 2 — Circular IK Computation
For each waypoint:
- compute IK
- warm‑start solver with previous result
- handle failures gracefully

### Acceptance
- ≥ 95% successful IK
- No large joint jumps
- Joint limits respected

### Deliverables
- `circular_ik.py`
- joint trajectory plot

---

## ▶️ Phase 3 — Execute Motion
Execute using ROS2 / MuJoCo / Custom Controller

### Requirements
- smooth motion
- configurable duration
- repeat at least 3 loops

### Deliverables
- working motion
- short video / GIF

---

## 🔎 Phase 4 — Validation & Proof
Must show:
1️⃣ EE path is circular  
2️⃣ Error acceptable

### Provide
- EE path plot
- RMSE tracking error
- Discussion on:
  - tracking
  - IK stability
  - workspace issues

---

## 📂 Submission Structure
```
circular_ik_assignment_<name>/
 ├─ src/
 │   ├─ circle_generator.py
 │   ├─ circular_ik.py
 │   └─ execution_script.py
 ├─ plots/
 │   ├─ circle_path.png
 │   ├─ joint_trajectories.png
 │   └─ ee_tracking_error.png
 ├─ video/
 │   └─ circular_motion.mp4
 └─ report.md
```

---

## 📄 report.md Must Include
- explanation of approach
- circle parameters
- solver description
- encountered problems
- evaluation

---

## ⭐ Bonus
- orientation follows tangent
- closed‑loop IK
- variable speed
- ellipse path

---

## 🕒 Suggested Timeline
| Phase | Time |
|------|------|
| Path Generation | 2h |
| IK Integration | 3–4h |
| Execution | 2h |
| Validation | 1–2h |

---

## ✅ Acceptance Checklist
- ☐ Circle correct
- ☐ IK continuous
- ☐ No joint blowups
- ☐ Stable execution
- ☐ Evidence provided

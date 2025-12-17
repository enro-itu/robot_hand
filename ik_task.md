# Assignment: Inverse Kinematics for a 3-Finger Dexterous Robot Hand (ROS 2)

## 0) Context (what is already implemented)
You are given a **working 3-finger robot hand simulation** package (`three_finger_hand`) with:

- URDF/Xacro description of a **3-finger hand**, each finger having **4 revolute joints**
- Gazebo simulation + ros2_control integration
- A **forward position controller** controlling all 12 joints
- A **GUI (Tkinter)** that directly commands joint positions (forward kinematics control)
- RViz visualization and Gazebo launch files

Currently, the system works purely in **joint space**: sliders → joint angles → hand motion.  
**Inverse kinematics (IK) is NOT implemented yet.**

This assignment extends the system by adding **Cartesian-space control using inverse kinematics**.

---

## 1) Goal
Design and implement an **Inverse Kinematics (IK) module** for the robot hand that allows:

- Controlling **finger tip positions in Cartesian space**
- Mapping desired fingertip poses → joint angles
- Executing grasp primitives using IK instead of hardcoded joint values

---

## 2) Robot Model Assumptions
- Number of fingers: **3**
- Joints per finger: **4 revolute joints**
- Total DOF: **12**
- Each finger is **kinematically independent**

---

## 3) Assignment Tasks

### Task 1 — Kinematic Modeling
For **one finger**:
1. Define the kinematic chain
2. Assign link lengths and joint axes
3. Derive forward kinematics

Deliverable: short explanation (PDF/MD).

---

### Task 2 — Inverse Kinematics Algorithm
Implement **numerical IK** (recommended):
- Jacobian transpose or damped least squares
- Respect joint limits
- Detect non-convergence

---

### Task 3 — ROS 2 IK Node
Create a node:
- Subscribes to desired fingertip pose
- Computes IK
- Publishes joint commands to the position controller

---

### Task 4 — GUI Extension
Extend the GUI with:
- Finger selector
- X/Y/Z inputs
- “Solve IK & Move” button

GUI must call IK solver, not send joint angles directly.

---

### Task 5 — IK-Based Grasp
Implement a pinch grasp using IK-defined fingertip targets.

---

## 4) Safety
- Joint limit enforcement
- Unreachable target detection
- Graceful failure

---

## 5) Deliverables
1. IK solver node
2. Updated GUI
3. Kinematics explanation
4. Demo recording or screenshots

---

## 6) Tests
- Single finger IK
- Joint limit test
- Multi-finger IK
- IK-based grasp

---

**This assignment upgrades the hand from joint-space control to true dexterous manipulation.**

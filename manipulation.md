## 🟦 Issue: 3-Finger Robot Hand Manipulation Demo (Circular IK)

**Labels:** robot-hand, manipulation, simulation, IK, milestone-jan25

---

### 🎯 Goal
Create a repeatable manipulation demo using the existing 3-finger robot hand simulation and the current circular IK implementation.

The demo should clearly show:
- Open → wrap → grasp → hold
- Stable behavior
- Clear control flow

This task focuses on understanding and structure, not perfection or ML.

---

### 📌 Given / Existing
- 3-finger robot hand simulation already exists
- A basic circular IK implementation already exists

You are expected to build on these, not redesign them.

---

## 🧩 Scope

### ✅ Included
- Simulation only (use the current simulator setup)
- 3-finger hand
- Single object (recommended: cylinder)
- Position-based control + grasp synergy

### ❌ Not included
- Machine learning
- Vision-based grasping
- Tactile / force optimization
- Dynamic object repositioning

---

## 🛠️ Tasks

### 1️⃣ Define the setup
Create a `definitions.md` file describing:
- Simulator used
- Object type and fixed pose
- Fingertip link/frame names
- What “circular IK” means in this project

---

### 2️⃣ Circular IK → trajectory
Turn the existing circular IK into a replayable trajectory generator:
- Input: object center + radius
- Output: time-based joint trajectory
- Must run consistently with the same initial state

---

### 3️⃣ Grasp synergy
Implement a single grasp parameter:

- g ∈ [0,1]
  - g = 0 → fully open
  - g = 1 → fully closed
- Map g linearly to finger joint angles

This is required for stability.

---

### 4️⃣ Manipulation demo (state machine)
Create one script/node that runs:

1. OPEN – hand fully open (~1s)
2. WRAP – execute circular IK trajectory (~2–3s)
3. CLOSE – ramp g from 0 → 1 (~1s)
4. HOLD – hold object ≥ 3 seconds
5. (Optional) small lift or movement

---

## ✅ Acceptance Criteria
- Object stays grasped for ≥ 3 seconds
- Demo is repeatable with same initial conditions
- Code structure is readable and modular

Bonus:
- Small lift or movement without dropping

---

## 📦 Deliverables
- Demo script/node
- Circular IK / trajectory code (cleaned)
- Launch file
- definitions.md
- README.md (how to run + notes)
- 30–60s screen recording of the demo

---

## 🧠 Expected Understanding
You should be able to explain:
- How commands reach the hand joints
- How circular IK affects fingertip motion
- Why the grasp succeeds or fails
- Why grasp synergy is used

---

## ⚠️ Notes
- Do not overcomplicate the task
- Do not jump into ML or vision
- Focus on clarity, stability, repeatability

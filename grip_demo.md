# Assignment: 3-Finger Hand — In-Hand Cube Manipulation (ROS2 Jazzy, Gazebo)

---

## Goal (Definition of Done)

You have a 3-finger robot hand simulation (already able to **grip**). You will:

1. spawn a **cube** with **different color shade per face**,
2. grasp it reliably,
3. manipulate it **in-hand** by commanding **orientation** (roll/pitch/yaw) and **angular velocity** (and optionally linear velocity),
4. log results and verify the cube reaches the commanded motion.

Success criteria:

* Hand grasps cube and holds ≥ 10 s without dropping (baseline).
* You can command **at least 3 orientations** (e.g., 0°, 90°, 180° yaw) while keeping grasp.
* You can command at least **two different angular velocities** (e.g., 0.5 rad/s and 1.0 rad/s) and observe rotation.

---

## Replace These Placeholders (One-Time Setup)

### Simulation / middleware

* `<GAZEBO_VARIANT>` = `gz-sim` (Harmonic/Ignition) **or** `gazebo-classic`
* `<BRIDGE_PKG>` = `ros_gz_bridge` (gz-sim) **or** `gazebo_ros_pkgs` (classic)

### Hand model

* `<HAND_MODEL_NAME>` = name of the hand entity in Gazebo
* `<HAND_BASE_LINK>` = base link frame
* `<FINGER_JOINTS>` = list of all finger joints used for manipulation
* `<GRIP_SERVICE_OR_ACTION>` = your existing “grip” interface (service/action/topic)

### Cube model

* `<CUBE_MODEL_NAME>` = `color_cube`
* `<CUBE_SIZE>` = `0.05` (meters) (example: 5 cm)
* `<CUBE_MASS>` = `0.2` kg (placeholder)
* `<CUBE_FRICTION_MU>` = 1.0 (placeholder)

### Topics (choose based on your sim stack)

* `<CUBE_POSE_TOPIC>` = cube pose in the world (e.g., model pose/state topic)
* `<CUBE_TWIST_TOPIC>` = cube twist in the world (optional)
* `<HAND_JOINT_STATES>` = `/joint_states`

---

## System Architecture (Recommended)

### Nodes

1. **Spawner / World bringup**

   * Starts Gazebo world and spawns hand + cube.

2. **Grasp manager**

   * Calls your existing grip function to close fingers and hold cube.

3. **In-hand manipulation controller (your code)**

   * Input: desired cube orientation (r/p/y) + desired angular velocity
   * Output: finger joint commands (position/velocity/effort) to achieve the motion while maintaining grasp

4. **State estimator (simulation truth)**

   * Reads cube pose/twist directly from Gazebo/bridge (no vision needed).

---

## PHASE 0 — Baseline Bringup (½ day)

### Task 0.1: Confirm hand control works

* Launch simulation with only the hand.
* Verify you can open/close the hand and hold a fixed pose.

**Deliverable:** `hand_bringup.launch.py`

### Task 0.2: Confirm joint interfaces

* Confirm finger joints are controllable (position/velocity/effort).
* Confirm `/joint_states` publishes.

**Deliverable:** `docs/hand_joints.md`

---

## PHASE 1 — Cube with Per-Face Colors (½–1 day)

### Task 1.1: Create cube model (SDF/URDF)

Create a cube with 6 distinct face colors/shades.

Recommended approach:

* Use an SDF with a single box collision.
* Use either:

  * 6 visuals (one per face) slightly offset, each with different material color, **or**
  * a single visual with a texture atlas (advanced, optional).

**Deliverables**

* `models/<CUBE_MODEL_NAME>/model.sdf`
* `models/<CUBE_MODEL_NAME>/materials/…` (if textures used)
* `docs/cube_colors.md` (list which face is which color)

### Task 1.2: Physics params

Set placeholders (tune later):

* mass `<CUBE_MASS>`
* friction `<CUBE_FRICTION_MU>`
* contact parameters (ERP/CFM or equivalent in your Gazebo variant)

**Acceptance check**

* Cube falls and rests stably on table.

---

## PHASE 2 — Spawn World + Cube Placement (½ day)

### Task 2.1: World file

Create a simple world:

* ground plane
* table (optional)
* good lighting

### Task 2.2: Spawn cube in front of the hand

* Place cube reachable by fingers.

**Deliverable:** `worlds/hand_cube.world` (or equivalent)

---

## PHASE 3 — Grasp Baseline (1 day)

### Task 3.1: Scripted grasp

Write a node/script that:

1. moves hand to pre-grasp pose (if needed)
2. calls `<GRIP_SERVICE_OR_ACTION>`
3. verifies cube is held

### Task 3.2: Held verification

Verify by monitoring:

* cube height doesn’t drop
* cube pose stays within a small region relative to hand

**Deliverables**

* `src/grasp_manager_node.py` (or cpp)
* `docs/grasp_test.md`

---

## PHASE 4 — Cube State Feedback (Simulation Truth) (½ day)

### Task 4.1: Subscribe to cube pose and twist

Depending on your sim:

* **gz-sim**: bridge model pose/twist to ROS2 topics
* **gazebo classic**: use model states or link states

You must provide the controller with:

* cube orientation (quaternion)
* cube angular velocity (optional but recommended)

**Deliverable:** `src/cube_state_node.py`

---

## PHASE 5 — In-Hand Manipulation Controller (2–4 days)

### Control objective

Given a command:

* desired orientation `R_des` (roll/pitch/yaw or quaternion)
* desired angular velocity `ω_des` (rad/s)

produce finger joint commands that:

* maintain contact / grasp force
* rotate cube toward `R_des` and/or track `ω_des`

### Task 5.1: Define command interface

Create topics:

* `/cube_cmd/orientation` (`geometry_msgs/PoseStamped` or quaternion message)
* `/cube_cmd/angular_velocity` (`geometry_msgs/TwistStamped` with only angular part)
* `/manipulation/enable` (`std_msgs/Bool`)

**Deliverable:** `docs/command_interface.md`

### Task 5.2: Start with simple “motion primitives” (recommended first)

Implement 3 primitive actions while holding cube:

* **Yaw+**: rotate cube clockwise in-hand
* **Yaw-**: rotate cube counter-clockwise
* **Roll**: rotate around another axis

Each primitive can be a small, periodic modulation of fingertip forces/positions.

**Deliverable:** `src/inhand_primitives_controller.py`

### Task 5.3: Closed-loop orientation tracking (PID on orientation error)

* Compute orientation error `e_R` between `R_current` and `R_des`
* Map that error to a desired object angular velocity `ω_obj`
* Map `ω_obj` to finger joint deltas (Jacobian-based if you have it; otherwise heuristic mapping per primitive)

Notes:

* Start with controlling **one axis (yaw)**, then expand to roll/pitch.

**Deliverable:** `src/inhand_orientation_controller.py`

### Task 5.4: Maintain grasp / force

If your hand already has a grip function:

* run it as a background “grip maintainer”
* superimpose small manipulation motions on top

Add safeguards:

* joint limits clamp
* max delta per step
* stop if cube slipping detected

**Deliverable:** `docs/safety_limits.md`

---

## PHASE 6 — Velocity Control Mode (1–2 days)

### Task 6.1: Track angular velocity

* Use cube measured angular velocity `ω_meas`
* Control `ω_meas → ω_des` via PID
* Drive primitives/mapping accordingly

**Acceptance check**

* Command `0.5 rad/s` yaw and observe stable rotation near that speed.

**Deliverable:** `src/inhand_velocity_controller.py`

---

## PHASE 7 — UI / Teleop for Commands (½ day)

### Task 7.1: Minimal teleop

Provide a simple interface:

* keyboard commands: `y`/`h` yaw ±, `r`/`f` roll ±, `p`/`l` pitch ±
* or a small PyQt panel with:

  * desired yaw angle slider
  * desired yaw rate slider
  * enable toggle

**Deliverable:** `src/inhand_teleop.py` (or `ui_inhand.py`)

---

## PHASE 8 — Logging & Evaluation (½ day)

### Task 8.1: Log data

Log at 50–100 Hz:

* cube orientation (quaternion)
* cube angular velocity
* commanded targets
* finger joint states

### Task 8.2: Metrics

* time to reach target orientation
* steady-state error (deg)
* average tracked angular velocity error
* drop/slip events

**Deliverables**

* `docs/test_report.md`
* bag files / plots

---

## Final Acceptance Demo Script

1. Launch world with hand + color cube
2. Grip cube (hold stable)
3. Rotate cube yaw to 90°
4. Rotate cube yaw back to 0°
5. Command yaw rate 0.5 rad/s for 5 seconds
6. Stop, hold stable

---

## Notes / Recommended Order

* Get **stable grasp** first.
* Implement **yaw manipulation only** next.
* Add **velocity tracking** after yaw position works.
* Add roll/pitch last.

---

## Optional Extensions

* Replace heuristic primitives with a Jacobian-based fingertip controller (if you can compute contact points).
* Add a “regrasp” behavior if cube slips.
* Add vision later (use face colors to estimate orientation visually), but **not needed** in simulation.

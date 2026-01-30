# Definitions for manupilation

## Simulator Environment

* **Simulator:** Gazebo Harmonic integrated with ROS 2 Jazzy.
* **Fixed Frame:** `world`.
* **Physics Engine:** ODE, Default physics engine in Gazebo.

## Target Object (Cylinder)

**Fixed Pose:**

* **Position (XYZ):** 0 0 0.7.
* **Orientation (RPY):** 0 0 0.

**Dimensions:**

* **Radius:**  m (5 cm).
* **Height:**  m (20 cm).

## Robot Hand Configuration

* **Robot Name:** `three_finger_hand`.
* **Fingertip Link Names:**
* `finger_1_distal`
* `finger_2_distal`
* `finger_3_distal`

## Circular IK Definition

* **Logic:** Within the scope of this project, IK is only **position based**.
* **Behavior:** IK node locates finger tips to selected 3 points around a created circle defined with a radius, axis, and center position.
* **Constraint:**  The orientation of the distal links is not controlled; the goal is only to reach the coordinates of the end points on the circle.

## Running manipulation demo

To run the existing manipulation demo, required files are simulation.launch.py, ik_solver_node.py, and manipulation_demo_node.py. Follow the commands below to run.

```bash
# Launch simulation as mentioned in the report file, but with a 'man:=true' flag.
ros2 launch three_finger_hand simulation.launch.py man:=true

# In another terminal which is on the launch dir, launch ik solver node.
python3 ik_solver_node.py

# In another terminal, launch manipulation demo node.
python3 manipulation_demo_node.py
```

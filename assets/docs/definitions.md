# Definitions for manupilation

## Simulator Environment

* **Simulator:** Gazebo Harmonic integrated with ROS 2 Jazzy.
* **Fixed Frame:** `world`.
* **Physics Engine:** ODE, Default physics engine in Gazebo.

## Target Object (Cylinder)

* **Fixed Pose:**
* **Position (XYZ):** .
* **Orientation (RPY):** .
* **Dimensions:**
* **Radius:**  m (5 cm).
* **Height:**  m (10 cm).

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

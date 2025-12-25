# ASSIGNMENT — 3-Finger Dexterous Robot Hand (Design + UI + Simulation)
**Project:** ENRO — Robotics R&D  
**Assigned to:** Eren  
**Reviewed by:** Emre / Team Lead  
**Date:** 09.12.2025

---

## 1. Objective
Develop a **3-finger dexterous robot hand**, inspired by the Shadow Robot x DeepMind hand:

https://shadowrobot.com/news/press-releases/shadow-robot-unveils-the-worlds-most-robust-dexterous-robot-hand-developed-in-partnership-with-google-deepmind/

Your tasks:

1. Use the provided Fusion 360 design files (or converted STEP/STL files).  
2. Assemble a **3‑finger hand**, with **120° finger spacing**.  
3. Create the **simulation-ready model** (URDF/Xacro).  
4. Build the **control UI**.  
5. Demonstrate the hand in simulation with joint movements and poses.

You do **not** need to redesign the parts—use the Fusion files provided.  
If conversion is needed, get help from the team.

---

## 2. Deliverables

### Mandatory
- 3D model assembled from provided Fusion files  
- `three_finger_hand.urdf.xacro`  
- Simulation launch files (RViz + Gazebo / Isaac Sim)  
- UI for joint control  
- Simulation demo video or screenshots  
- `docs/three_finger_hand_report.md`

### Optional
- Gesture presets (pinch, tripod, open/close cycle)  
- RViz panel plugin  
- Grasp animation  

---

## 3. Inputs You Will Receive
You will be given:

- Fusion 360 model files  
- A demonstration video  
- A reference image  

Convert formats (STEP/STL/OBJ) if required with help from the team.

---

## 4. Tasks (Step-by-Step)

### **Task 1 — Study the reference**
Understand the Shadow Robot hand’s:
- Dexterity  
- Joint structure  
- Grasp ability  

---

### **Task 2 — CAD Assembly**
- Import Fusion 360 files  
- Adjust axes & origin  
- Fix scaling and mesh orientation  
- Assemble 3 fingers at **120° spacing**  
- Export meshes as STL/OBJ

Output directory:

```
hand_description/meshes/
```

---

### **Task 3 — URDF / Xacro Model**
Define:
- All links  
- All joints  
- Joint limits  
- Visual + collision meshes

Use modular Xacro macros.

Final URDF path:

```
hand_description/urdf/three_finger_hand.urdf.xacro
```

---

### **Task 4 — Simulation Integration**
Create:

- `display.launch.py` (RViz)  
- `simulation.launch.py` (Gazebo/Isaac)

Simulation requirements:
- All joints work  
- No physics explosions  
- Stable poses  

---

### **Task 5 — UI**
Use PyQt5 or PySide2.

UI must include:
- Sliders for each joint  
- “Open Hand” button  
- “Close Hand” button  
- Pose buttons  
- Real-time joint feedback  

Publish to:

```
/three_finger_hand_controller/commands
```

---

### **Task 6 — Demo**
Show:
- All fingers closing  
- Two-finger pinch  
- Three-finger grasp  
- Open/close animation  

---

## 5. Acceptance Criteria

### CAD  
- Fusion files imported cleanly  
- 3-finger model accurate and aligned

### URDF/Xacro  
- No missing meshes  
- Joint orientations correct  
- Launch files working

### Simulation  
- Joints move smoothly  
- Stable behavior  
- Poses executed correctly

### UI  
- Sliders functional  
- Buttons functional  
- Commands published correctly

### Documentation  
- README with instructions  
- Explanation of joints and setup  

### Delivery  
- Branch name: `feature/three-finger-hand-eren`  

---

## 6. Notes for Eren
- Don’t waste time redrawing parts — use the provided Fusion files.  
- Convert formats only if needed.  
- Ask to the team for conversion help.  
- Work in this order: **CAD → URDF → Simulation → UI → Demo**.  
- Keep everything modular and clean.

---

**End of file.**

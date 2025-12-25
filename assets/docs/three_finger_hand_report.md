# Report for assignment - 3-Finger Dexterous Robot Hand (Design + UI + Simulation)

**Author**: Eren

This project contains a 3-finger robot, its UI, simulation and design. This robot is developed with Ros2 Jazzy, simulated with Gazebo Harmonic, and has a UI made with Tkinter.

## Usage

**Note**: Before starting, please keep in mind that the commands below is valid for bash. If you use Powershell or another shell except bash, check commands before running.

### Preliminary

After cloning project, firstly you should install required libraries and run some scripts to automatically generate required files. Here are how to do.

```bash
# In the project directory
colcon build
cd src/three_finger_hand/launch
python3 -m venv .venv # Choosing ".venv" name is recommended to avoid gitignore problems
source .venv/bin/activate
pip install PyYAML setuptools jinja2 typeguard numpy ikpy
```

### Starting RViz to preview the robot

```bash
# In the project directory
source install/setup.bash
ros2 launch three_finger_hand display.launch.py
```

### Starting Gazebo simulation

```bash
# In the project directory
source install/setup.bash
ros2 launch three_finger_hand simulation.launch.py
```

### Running IK solver

Be sure running Gazebo Sim before running this.

```bash
# In the project directory
source install/setup.bash
cd src/three_finger_hand/launch
source .venv/bin/activate
python3 ik_solver_node.py
```

IK solver could be run with GUI. Also could be run with a command like below.

```bash
ros2 topic pub --once /finger_1/goal_pose geometry_msgs/msg/Point "{x: 0.045, y: 0.015, z: 0.19}"
```

### Opening GUI

Be sure running Gazebo Sim and IK solver before running this.

```bash
# In the project directory
source install/setup.bash
cd src/three_finger_hand/launch
source .venv/bin/activate
python3 controller_gui.py
```

## Data and media related to the robot

You can find related Fusion files in the ```parts``` folder, and STL files in the ```src/three_finger_hand/meshes``` directory. Also, the ```assets``` folder includes media related to robot. Here are some screenshots while robot running.

* Screenshot from robot in RViz

![Screenshot from RViz](../media/screenshot%20from%20rviz.png)

* Screenshot from robot in Gazebo and GUI control tool

![Screenshot from Gazebo](../media/screenshot%20from%20gazebo%20sim%20and%20control%20ui.png)

* Video from Gazebo Sim showing demo movements

[![Demo movement video](../media/screenshot%20from%20gazebo%20sim%20and%20control%20ui.png)](../media/screencast%20from%20gazebo%20sim.mp4)

* Video from Gazebo Sim showing IK capabilities

[![Demo IK video](../media/screenshot%20from%20gazebo%20sim%20and%20control%20ui.png)](../media/screencast%20from%20gazebo%20sim%20and%20ik.mp4)

* Video from Gazebo Sim showing circular IK capabilities

[![Demo circular IK video](../media/screenshot%20from%20circular%20ik%20demo.png)](../media/screencast%20from%20circular%20ik%20demo.mp4)

* Photo from the robot in Fusion

![Photo from Fusion](../media/finger%20in%20fusion.jpeg)

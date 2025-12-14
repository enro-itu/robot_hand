# Report for assignment - 3-Finger Dexterous Robot Hand (Design + UI + Simulation)

**Author**: Eren

This project contains a 3-finger robot, its UI, simulation and design. This robot is developed with Ros2 Jazzy, simulated with Gazebo Harmonic, and has a UI made with Tkinter.

## Usage

After cloning project, and being sure installing required programs, go to the project directory and open 3 terminals. Then, run the code below.

```bash
# Terminal 1 (To view in RViz, optional)
source install/setup.bash
colcon build # 1-time-required, additionally required after editing
ros2 launch three_finger_hand display.launch.py # Opens config.rviz automatically

# Terminal 2 (To simulate in Gazebo)
source install/setup.bash
colcon build # Run if skipped Terminal 1
ros2 launch three_finger_hand simulation.launch.py

# Terminal 3 (To manage the robot using GUI)
cd src/three_finger_hand/launch/controller_gui.py
```

## Data and media related to the robot

You can find related Fusion files in the ```parts``` folder, and STL files in the ```src/three_finger_hand/meshes``` directory. Also, the ```assets``` folder includes media related to robot. Here are some screenshots while robot running.

* Screenshot from robot in RViz

![Screenshot from RViz](../media/screenshot%20from%20rviz.png)

* Screenshot from robot in Gazebo and GUI control tool

![Screenshot from Gazebo](../media/screenshot%20from%20gazebo%20sim%20and%20control%20ui.png)

* Video from Gazebo Sim showing demo movements

[![Demo movement video](../media/screenshot%20from%20gazebo%20sim%20and%20control%20ui.png)](../media/screencast%20from%20gazebo%20sim.mp4)

* Photo from a finger in Fusion

![Photo from Fusion](../media/finger%20in%20fusion.jpeg)

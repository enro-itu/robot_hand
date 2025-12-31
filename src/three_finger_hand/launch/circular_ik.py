import numpy as np
import ikpy.chain
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def run_circular_ik():
    try:
        # Create temporary URDF
        pkg_share = get_package_share_directory('three_finger_hand')
        xacro_path = os.path.join(pkg_share, 'urdf', 'three_finger_hand.urdf.xacro')
        urdf_path = "/tmp/circular_hand_debug.urdf"

        # Convert XACRO to URDF
        subprocess.run(f"xacro {xacro_path} > {urdf_path}", shell=True, check=True)
        print(f"URDF created: {urdf_path}")
    except Exception as e:
        print(f"URDF creating error: {e}")
        return

    # Load waypoints
    try:
        waypoints = np.load('src/waypoints.npy')
        print(f"{len(waypoints)} waypoint loaded")
    except FileNotFoundError:
        print("Error: src/waypoints.npy not found, run circle_generator.py first")
        return

    # 3. Load IK chain
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["palm"], name="finger_1")

    trajectory_12dof = []

    # Warm-start
    last_ik_res = [0.0] * len(chain.links)

    success_count = 0
    print("Starting calculating...")

    for i, target in enumerate(waypoints):
        # IK solving
        ik_res = chain.inverse_kinematics(target, initial_position=last_ik_res)

        computed_pos = chain.forward_kinematics(ik_res)[:3, 3]
        dist = np.linalg.norm(target - computed_pos)

        if dist < 0.02: # 2 cm tolerance
            success_count += 1
            last_ik_res = ik_res # Keep for warm-start

        full_cmd = [0.0] * 12
        full_cmd[0:4] = ik_res[1:5].tolist()
        trajectory_12dof.append(full_cmd)

    os.makedirs('src', exist_ok=True)
    np.save('src/joint_trajectory.npy', np.array(trajectory_12dof))

    success_rate = (success_count / len(waypoints)) * 100
    print(f"Success rate: %{success_rate:.2f}")

if __name__ == "__main__":
    run_circular_ik()

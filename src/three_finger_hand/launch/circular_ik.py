import numpy as np
import ikpy.chain
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def run_circular_ik():
    # Manually generate temporary URDF with debug info
    try:
        pkg_name = 'three_finger_hand'
        pkg_share = get_package_share_directory(pkg_name)
        xacro_file = os.path.join(pkg_share, 'urdf', 'three_finger_hand.urdf.xacro')
        urdf_output = "/tmp/circular_hand_debug.urdf"

        subprocess.run(['xacro', xacro_file, '-o', urdf_output], check=True)
        print(f"Temporary URDF generated at {urdf_output}")
    except subprocess.CalledProcessError as e:
        print(f"Error generating URDF: {e}")
        return

    # Load URDF and Waypoints
    urdf_path = "/tmp/circular_hand_debug.urdf"
    waypoints = np.load('src/waypoints.npy')

    # Load Chain and Mask Fixed Joints
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["palm"], name="finger_1")

    # Check total number of links in the chain
    n_links = len(chain.links)
    active_mask = [False] * n_links
    for i in range(1, n_links - 1):
        active_mask[i] = True
    chain.active_links_mask = active_mask

    print(f"Chain Structure: {[link.name for link in chain.links]}")
    print(f"Active Joint Mask: {chain.active_links_mask}")

    trajectory_12dof = []
    last_ik_res = [0.0] * n_links
    success_count = 0

    print("\nCalculation started...")
    for i, target in enumerate(waypoints):
        # IK Solution
        ik_res = chain.inverse_kinematics(target, initial_position=last_ik_res)

        # Verification with FK
        computed_pos = chain.forward_kinematics(ik_res)[:3, 3]
        dist = np.linalg.norm(target - computed_pos)

        # Error analysis for the first point
        if i == 0:
            print(f"First Point Analysis:")
            print(f"  Target: {target}")
            print(f"  Reached: {computed_pos}")
            print(f"  Distance Error: {dist:.4f}m")

        # 1 cm tolerance
        if dist < 0.01:
            success_count += 1
            last_ik_res = ik_res

        # 12-DOF Packaging
        full_cmd = [0.0] * 12
        # Make sure ik_res[1:5] corresponds to your movable joints
        full_cmd[0:4] = ik_res[1:5].tolist()
        trajectory_12dof.append(full_cmd)

    success_rate = (success_count / len(waypoints)) * 100
    print(f"\nCompleted: Success Rate %{success_rate:.2f}")

    if success_count > 0:
        np.save('src/joint_trajectory.npy', np.array(trajectory_12dof))
        print("Trajectory saved.")
if __name__ == "__main__":
    run_circular_ik()

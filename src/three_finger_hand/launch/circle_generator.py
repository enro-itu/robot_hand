import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def generate_circle_points(center, normal, radius, N):
    center = np.array(center)
    normal = np.array(normal) / np.linalg.norm(normal)

    if abs(normal[0]) < 0.9:
        u = np.cross(normal, [1, 0, 0])
    else:
        u = np.cross(normal, [0, 1, 0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    thetas = np.linspace(0, 2 * np.pi, N, endpoint=False)
    points = [center + radius * np.cos(t) * u + radius * np.sin(t) * v for t in thetas]
    return np.array(points)

def visualize_circle(points, center, normal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], "-o", label='Circlular path')
    ax.scatter(center[0], center[1], center[2], color='red', label='Center')
    ax.quiver(center[0], center[1], center[2], normal[0], normal[1], normal[2], length=0.1, color='green', label='Normal')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Example run: circle_generator.py -c 0.6 0 0.19 -n 0 0 1 -r 0.06 -p 50

    parser = argparse.ArgumentParser(description="Generate circular waypoints from CLI input.")
    parser.add_argument("--center", "-c", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                        help="Center of the circle (x y z)")
    parser.add_argument("--normal", "-n", type=float, nargs=3, required=True, metavar=("NX", "NY", "NZ"),
                        help="Normal vector of the plane (nx ny nz)")
    parser.add_argument("--radius", "-r", type=float, required=True, help="Radius of the circle")
    parser.add_argument("--N", "-p", type=int, required=True, help="Number of waypoints along the circle")
    args = parser.parse_args()

    if args.radius <= 0:
        raise ValueError("radius must be positive")
    if args.N < 3:
        raise ValueError("N must be at least 3 to form a circle")

    center = args.center
    normal = args.normal
    radius = args.radius
    N = args.N

    waypoints = generate_circle_points(center, normal, radius, N)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'src')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'waypoints.npy')
    np.save(output_path, waypoints)
    print(f"{N} waypoints saved to '{output_path}'.")
    visualize_circle(waypoints, center, normal)

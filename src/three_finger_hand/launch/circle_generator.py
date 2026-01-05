import numpy as np
import os
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

    thetas = np.linspace(0, 2 * np.pi, N)
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
    # Sample appropriate parameters, please preview by running
    # waypoint_visualiser.py to be sure the circle looks correct
    center = [0.0, 0.0, 0.19]
    normal = [0, 0, 1]
    radius = 0.06
    N = 50

    waypoints = generate_circle_points(center, normal, radius, N)

    os.makedirs('src', exist_ok=True)
    np.save('src/waypoints.npy', waypoints)
    print(f"{N} waypoints saved to 'src/waypoints.npy'.")
    visualize_circle(waypoints, center, normal)

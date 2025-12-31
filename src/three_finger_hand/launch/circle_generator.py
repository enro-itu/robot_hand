import numpy as np
import os

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

if __name__ == "__main__":
    center = [0.04, 0.0, 0.03]
    normal = [0, 0, 1]
    radius = 0.03
    N = 50

    waypoints = generate_circle_points(center, normal, radius, N)

    os.makedirs('src', exist_ok=True)
    np.save('src/waypoints.npy', waypoints)
    print(f"{N} waypoints saved to 'src/waypoints.npy'.")

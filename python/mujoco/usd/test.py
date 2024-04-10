import open3d as o3d
import numpy as np

import open3d as o3d
import numpy as np

# Define parameters
radius = 1.0  # Radius of the hemisphere
resolution = 20  # Number of points per circle
theta_steps = 20  # Number of vertical steps (slices)
phi_steps = 20  # Number of horizontal steps

# Generate points for the hemisphere
points = []
for i in range(phi_steps + 1):
    phi = np.pi / 2 * i / phi_steps
    for j in range(theta_steps + 1):
        theta = 2 * np.pi * j / theta_steps
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        points.append([x, y, z])

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Convert point cloud to mesh - alpha
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=10)

# Convert point cloud to mesh - ball pivoting
# radii = [0.005, 0.01, 0.02, 0.04]
# pcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     pcd, o3d.utility.DoubleVector(radii)
# )

# Visualize the mesh
o3d.visualization.draw_geometries([mesh])

# Visualize the point cloud
# o3d.visualization.draw_geometries([pcd])
import open3d as o3d
import numpy as np

# Load your XYZ file
point_cloud = o3d.io.read_point_cloud("pointcloud.xyz")

    # Define the number of nearest neighbors to connect each point to
k = 10

    # Compute k-nearest neighbors for each point
kdtree = o3d.geometry.KDTreeFlann(point_cloud)
lines = []
for i in range(len(point_cloud.points)):
        [k, idx, _] = kdtree.search_knn_vector_3d(point_cloud.points[i], k)
        for j in idx:
            if j != i:
                lines.append([i, j])

    # Create a LineSet to connect the points with lines
line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),
        lines=o3d.utility.Vector2iVector(lines),
)

    # Visualize the point cloud and lines
o3d.visualization.draw_geometries([point_cloud, line_set])

import serial
import math
import numpy as np
import open3d as o3d

# Constants
SAMPLES = 32
NUM_SCANS = 3
Z_INCREMENT = 10

# Variables
pi = math.pi
z = 0 # z value for the point cloud (cartesian)

# Establish serial connection
s = serial.Serial('COM4', 115200)

print("Opening:", s.name)

# Reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

s.write(b'1') # send 1 for the startup handshake

vals = []
f = open("./demo.xyz", "w")

try:
    stm = "Start Data Transfer (0 = false, 1 = true): "
    transferData = input(stm)[0]
    
    if(transferData):
        s.write(b'1') # send 1 for the data transfer handshake
        
    for i in range(SAMPLES * NUM_SCANS):
        # read the data from the serial port
        val = int(s.readline().decode()) 

        if((i % SAMPLES) == 0 and i != 0):
            z += Z_INCREMENT # increment the z value by 10 cm for each scan
        
        # convert polar to cartesian and scale the data to cm (default readings are in mm)
        x, y = val * math.sin(pi*i/16), val * math.cos(pi*i/16) 
        x, y, = x/10, y/10

        vals.append([x, y, z])
    
    for a in range(len(vals)):
        # write the data to the file in the format x y z
        string = '%s %s %s\n' % (vals[a][0], vals[a][1], vals[a][2])
        f.write(string)
             
    s.close()
    f.close()

    # Load your XYZ file
    point_cloud = o3d.io.read_point_cloud("demo.xyz")

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

       
except KeyboardInterrupt:
    print("Closing: " + s.name)
    s.close()
    f.close()

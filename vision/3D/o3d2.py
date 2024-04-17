import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


dataset = o3d.data.OfficePointClouds()
pcd = o3d.io.read_point_cloud(dataset.paths[0])
print(pcd)    
o3d.visualization.draw_geometries([pcd],
                                   zoom=0.3412,
                                   front=[0.4257, -0.2125, -0.8795],
                                   lookat=[2.6172, 2.0475, 1.532],
                                   up=[-0.0694, -0.9768, 0.2024])


import open3d as o3d

def your_update_function(vis):
    # Your update routine
    vis.update_geometry(cloud)
    vis.update_renderer()
    vis.poll_events()

dataset = o3d.data.OfficePointClouds()
cloud = o3d.io.read_point_cloud(dataset.paths[0])

# Create a visualizer
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.register_key_callback(ord("k"), your_update_function)
vis.add_geometry(cloud)

# Start the visualization loop
vis.run()

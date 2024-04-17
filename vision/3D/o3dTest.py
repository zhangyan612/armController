import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

dataset = o3d.data.LivingRoomPointClouds()
pcds = []
for pcd_path in dataset.paths:
    pcds.append(o3d.io.read_point_cloud(pcd_path))
print(pcds)

dataset = o3d.data.DemoICPPointClouds()
pcd0 = o3d.io.read_point_cloud(dataset.paths[0])
pcd1 = o3d.io.read_point_cloud(dataset.paths[1])
pcd2 = o3d.io.read_point_cloud(dataset.paths[2])

# # point cloud
# print("Load a ply point cloud, print it, and render it")
# ply_point_cloud = o3d.data.PLYPointCloud()
# pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])


# # mesh

# bunny = o3d.data.BunnyMesh()
# mesh = o3d.io.read_triangle_mesh(bunny.path)
# mesh.compute_vertex_normals()

# o3d.visualization.draw_geometries([mesh])
# pcd = mesh.sample_points_uniformly(number_of_points=500)
# o3d.visualization.draw_geometries([pcd])


# # depth to point cloud
# print("Read Redwood dataset")
# redwood_rgbd = o3d.data.SampleRedwoodRGBDImages()
# color_raw = o3d.io.read_image(redwood_rgbd.color_paths[0])
# depth_raw = o3d.io.read_image(redwood_rgbd.depth_paths[0])
# rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#     color_raw, depth_raw)
# print(rgbd_image)


# plt.subplot(1, 2, 1)
# plt.title('Redwood grayscale image')
# plt.imshow(rgbd_image.color)
# plt.subplot(1, 2, 2)
# plt.title('Redwood depth image')
# plt.imshow(rgbd_image.depth)
# plt.show()


# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
#     rgbd_image,
#     o3d.camera.PinholeCameraIntrinsic(
#         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
# # Flip it, otherwise the pointcloud will be upside down
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# o3d.visualization.draw_geometries([pcd])



# # depth to point cloud 2
print("Read SUN dataset")
sun_rgbd = o3d.data.SampleSUNRGBDImage()
color_raw = o3d.io.read_image(sun_rgbd.color_path)
depth_raw = o3d.io.read_image(sun_rgbd.depth_path)
rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)
print(rgbd_image)

# plt.subplot(1, 2, 1)
# plt.title('SUN grayscale image')
# plt.imshow(rgbd_image.color)
# plt.subplot(1, 2, 2)
# plt.title('SUN depth image')
# plt.imshow(rgbd_image.depth)
# plt.show()

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

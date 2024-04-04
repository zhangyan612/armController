import open3d as o3d

path = r'C:/Users/yanzh/Desktop/ROBOT/pointcloud.ply' # software export
# path = "point_cloud.ply"  #save code
# path = "point_cloud2.ply"  #create code
# path = "pointcloud3.ply"  # py viewer code 

def display_point_cloud_from_ply_file():
    pcd = o3d.io.read_point_cloud(path)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud Visualizer",
                      width=800, height=800)
    vis.add_geometry(pcd)
    render_opt = vis.get_render_option()
    render_opt.point_size = 2

    while True:
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

display_point_cloud_from_ply_file()

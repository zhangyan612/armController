import numpy as np
import open3d as o3d

class Viewer3D(object):

    def __init__(self, title):
        app = o3d.visualization.gui.Application.instance
        app.initialize()

        self.main_vis = o3d.visualization.O3DVisualizer(title)
        app.add_window(self.main_vis)
        
        self.setup_depth_streaming()
        self.setup_point_clouds()
        self.setup_o3d_scene()

    def setup_depth_streaming(self):
        # TODO: setup your depth / point cloud streaming source here
        pass

    def setup_point_clouds(self):
        # setup an empty point cloud to be later populated with live data
        # self.point_cloud_o3d = o3d.geometry.PointCloud()
        # the name is necessary to remove from the scene
        self.point_cloud_o3d_name = "point cloud"
        dataset = o3d.data.OfficePointClouds()
        self.point_cloud_o3d = o3d.io.read_point_cloud(dataset.paths[0])

    def update_point_clouds(self):
        # update your point cloud data here: convert depth to point cloud / filter / etc.
        pass

    def setup_o3d_scene(self):
        self.main_vis.add_geometry(self.point_cloud_o3d_name, self.point_cloud_o3d)
        self.main_vis.reset_camera_to_default()
        # center, eye, up
        self.main_vis.setup_camera(60,
                                    [4, 2, 5],
                                    [0, 0, -1.5],
                                    [0, 1, 0])

    def update_o3d_scene(self):
        self.main_vis.remove_geometry(self.point_cloud_o3d_name)
        self.main_vis.add_geometry(self.point_cloud_o3d_name, self.point_cloud_o3d)

    def run_one_tick(self):
        app = o3d.visualization.gui.Application.instance
        tick_return = app.run_one_tick()
        if tick_return:
            self.main_vis.post_redraw()
        return tick_return

viewer3d = Viewer3D("H&M point cloud")

try:
    while True:
        # Step 1) Perturb the cloud with a random walk to simulate an actual read
        # (based on https://github.com/isl-org/Open3D/blob/master/examples/python/visualization/multiple_windows.py)
        viewer3d.update_point_clouds()
        # Step 2) Update the cloud and tick the GUI application
        viewer3d.update_o3d_scene()
        viewer3d.run_one_tick()
except Exception as e:
    print(e)

import pyrealsense2 as rs
import numpy as np

def get_realsense_scene_points_and_colors():
    """
    Retrieves 3D point cloud and color data from a RealSense camera.

    Returns:
        tuple: A tuple containing scene points and colors.
    """

    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert depth data to point cloud
        points = rs.pointcloud()
        points.map_to(color_frame)
        points_data = points.calculate(depth_frame)

        # Get points and colors
        points = np.asanyarray(points_data.get_vertices())
        colors = np.asanyarray(points_data.get_texture_coordinates()) * 255
        colors = colors.astype(np.uint8)

        return points, colors

    finally:
        # Stop streaming
        pipeline.stop()

# Example usage
points, colors = get_realsense_scene_points_and_colors()

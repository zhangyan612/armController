import pyrealsense2 as rs
import numpy as np
# import open3d as o3d
from matplotlib import pyplot as plt
import cv2

class DepthCamera:
    def __init__(self, resolution_width, resolution_height):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        # Get depth scale of the device
        self.depth_scale =  depth_sensor.get_depth_scale()
            # Create an align object
        align_to = rs.stream.color

        self.align = rs.align(align_to)
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print("device product line:", device_product_line)
        config.enable_stream(rs.stream.depth,  resolution_width,  resolution_height, rs.format.z16, 6)
        config.enable_stream(rs.stream.color,  resolution_width,  resolution_height, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(config)

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame
    
    def get_depth_scale(self):
        """
        "scaling factor" refers to the relation between depth map units and meters; 
        it has nothing to do with the focal length of the camera.
        Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
        """
        return self.depth_scale

    def release(self):
        self.pipeline.stop()                                                                                      



def depth2PointCloud(depth, rgb, depth_scale, clip_distance_max):
    
    intrinsics = depth.profile.as_video_stream_profile().intrinsics
    depth = np.asanyarray(depth.get_data()) * depth_scale # 1000 mm => 0.001 meters
    rgb = np.asanyarray(rgb.get_data())
    rows,cols  = depth.shape

    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)

    valid = (depth > 0) & (depth < clip_distance_max) #remove from the depth image all values above a given value (meters).
    valid = np.ravel(valid)
    z = depth 
    x =  z * (c - intrinsics.ppx) / intrinsics.fx
    y =  z * (r - intrinsics.ppy) / intrinsics.fy
   
    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]
    
    r = np.ravel(rgb[:,:,0])[valid]
    g = np.ravel(rgb[:,:,1])[valid]
    b = np.ravel(rgb[:,:,2])[valid]
    
    pointsxyzrgb = np.dstack((x, y, z, r, g, b))
    pointsxyzrgb = pointsxyzrgb.reshape(-1,6)

    return pointsxyzrgb



def create_point_cloud_file2(vertices, filename):
    ply_header = '''ply
  format ascii 1.0
  element vertex %(vert_num)d
  property float x
  property float y
  property float z
  property uchar red
  property uchar green
  property uchar blue
  end_header
  '''
    with open(filename, 'w') as f:
        f.write(ply_header %dict(vert_num=len(vertices)))
        np.savetxt(f,vertices,'%f %f %f %d %d %d')


resolution_width, resolution_height = (640, 480)
clip_distance_max = 3.500 ##remove from the depth image all values above a given value (meters).

def main():

    Realsensed435Cam = DepthCamera(resolution_width, resolution_height)

    depth_scale = Realsensed435Cam.get_depth_scale()

    while True:

        ret , depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
        
        points_xyz_rgb = depth2PointCloud(depth_raw_frame, color_raw_frame, depth_scale, clip_distance_max)
        create_point_cloud_file2(points_xyz_rgb,"point_cloud2.ply")
                
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
   
        cv2.imshow("Frame",  color_frame )
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.imwrite("frame_color.png", color_frame)
            plt.imsave("frame_plt.png", depth_frame)
            break
    
    Realsensed435Cam.release() # release rs pipeline


if __name__ == '__main__':
    main()

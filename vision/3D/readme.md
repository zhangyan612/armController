

Ways to merge point cloud 
https://hatchjs.com/iterative-closest-point-python/

Iterative Closest Point


Hi @TheNemo05 If you are able to make use of commercial software then there are D455-compatible products that can perform the capture. These include RecFusion (Windows) and DotProduct Dot3D (Windows or Android).

https://www.recfusion.net/
https://www.dotproduct3d.com/

If you are able to use C++ then the RealSense SDK's rs-kinfu C++ open-source example program may be helpful. It enables a single camera to be moved around a scene, progressively building up a point cloud scan with frame fusion. Once you are satisfied with the level of detail on the scan then you can export the pointcloud to a .ply format pointcloud data file.

https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv/kinfu

If you are using Python then I would recommend performing the reconstruction with Open3D and its RealSense support.

http://www.open3d.org/docs/release/tutorial/sensor/realsense.html


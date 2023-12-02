# pip install opencv-python numpy PyOpenGL

import cv2
import numpy as np
import time
import struct
import OpenGL.GL as gl
import OpenGL.GLUT as glut

max_dist = 2.5
min_dist = 0

def colorize_pointcloud(points):
    channels = cv2.split(points)
    color = channels[2]
    min = min_dist
    max = max_dist
    color = cv2.convertScaleAbs(color, alpha=255 / (max - min), beta=-255 * min / (max - min))
    color = cv2.applyColorMap(color, cv2.COLORMAP_JET)
    return color

def draw_kinfu_pointcloud(points, normals):
    color = colorize_pointcloud(points)
    gl.glLoadIdentity()
    gl.glPushAttrib(gl.GL_ALL_ATTRIB_BITS)
    gl.glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1)
    gl.glClear(gl.GL_DEPTH_BUFFER_BIT)
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPushMatrix()
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glPushMatrix()
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_COLOR_MATERIAL)
    gl.glEnable(gl.GL_LIGHTING)
    gl.glEnable(gl.GL_LIGHT0)
    gl.glBegin(gl.GL_POINTS)
    for i in range(points.shape[0]):
        x, y, z = points[i]
        nx, ny, nz = normals[i]
        r, g, b = color[i]
        gl.glColor3ub(r, g, b)
        gl.glNormal3f(nx, ny, nz)
        gl.glVertex3f(x, y, z)
    gl.glEnd()
    gl.glPopMatrix()
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPopMatrix()
    gl.glPopAttrib()

def export_to_ply(points, normals):
    fname = time.strftime("%m%d%y %H%M%S.ply", time.localtime())
    print("exporting to", fname)
    color = colorize_pointcloud(points)
    with open(fname, 'wb') as f:
        f.write(b"ply\n")
        f.write(b"format binary_little_endian 1.0\n")
        f.write(b"comment pointcloud saved from Realsense Viewer\n")
        f.write(b"element vertex %d\n" % points.shape[0])
        f.write(b"property float32 x\n")
        f.write(b"property float32 y\n")
        f.write(b"property float32 z\n")
        f.write(b"property float32 nx\n")
        f.write(b"property float32 ny\n")
        f.write(b"property float32 nz\n")
        f.write(b"property uchar red\n")
        f.write(b"property uchar green\n")
        f.write(b"property uchar blue\n")
        f.write(b"end_header\n")
        for i in range(points.shape[0]):
            x, y, z = points[i]
            nx, ny, nz = normals[i]
            r, g, b = color[i]
            f.write(struct.pack('ffffffccc', x, y, z, nx, ny, nz, r, g, b))



# point cloud

# The motion estimation algorithm
# This stage of the demo uses the ICP algorithm to guess the movement of the camera without any motion sensor (also known as SLAM). It has been inspired by the paper KinectFusion: Real-time 3D Reconstruction and Interaction Using a Moving Depth Camera, which implements a similar version of the algorithm optimized for GPUs. Thanks to this design, it's able to process the frames in real-time even on a relatively weak GPU.


# https://github.com/intel/depthcamera-3d-model-web-demo
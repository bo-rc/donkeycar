'''
Author: Tawn Kramer
File: realsense2.py
Date: April 14 2019
Notes: Parts to input data from Intel Realsense 2 cameras
'''
import time
import cv2
import logging

import numpy as np
import pyrealsense2 as rs
import math as m

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])


class RS_T265_StereoRectified(object):
    '''
    The Intel Realsense T265 camera is a device which uses an imu, twin fisheye cameras,
    and an Movidius chip to do sensor fusion and emit a world space coordinate frame that 
    is remarkably consistent.
    '''

    def __init__(self, image_w=240, fov=120):
        
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
        cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

        # Set up a mutex to share data between threads 
        from threading import Lock
        self.frame_mutex = Lock()
        self.frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }

        def callback(frame):
            if frame.is_frameset():
                frameset = frame.as_frameset()
                f1 = frameset.get_fisheye_frame(1).as_video_frame()
                f2 = frameset.get_fisheye_frame(2).as_video_frame()
                left_data = np.asanyarray(f1.get_data())
                right_data = np.asanyarray(f2.get_data())
                ts = frameset.get_timestamp()
                self.frame_mutex.acquire()
                self.frame_data["left"] = left_data
                self.frame_data["right"] = right_data
                self.frame_data["timestamp_ms"] = ts
                self.frame_mutex.release()

        # Start streaming with our callback
        self.pipe.start(cfg, callback)
        time.sleep(0.5)

        profiles = self.pipe.get_active_profile()
        streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}
        
        # Print information about both cameras
        print(">>>Cal::Intrinsics::Left camera:",  intrinsics["left"])
        print(">>>Cal::Intrinsics::Right camera:", intrinsics["right"])

        (R, T) = get_extrinsics(streams["left"], streams["right"])
        print(">>>Cal::relativeExtrinsics:", (R,T))

        window_size = 5
        self.min_disp = 0
        self.num_disp = 112 - self.min_disp
        self.max_disp = self.min_disp + self.num_disp
        self.stereo = cv2.StereoSGBM_create(minDisparity = self.min_disp,
                                   numDisparities = self.num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)
        
        # Translate the intrinsics from librealsense into OpenCV
        K_left  = camera_matrix(intrinsics["left"])
        D_left  = fisheye_distortion(intrinsics["left"])
        K_right = camera_matrix(intrinsics["right"])
        D_right = fisheye_distortion(intrinsics["right"])
       
        # We calculate the undistorted focal length:
        #
        #         h
        # -----------------
        #  \      |      /
        #    \    | f  /
        #     \   |   /
        #      \ fov /
        #        \|/

        print("initializing FoV: ",fov, " Width px: ", image_w)
        stereo_fov_rad = fov * (m.pi/180)  # 110 degree desired fov
        stereo_height_px = image_w # use image_w to initialize height
        stereo_focal_px = stereo_height_px/2 / m.tan(stereo_fov_rad/2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras
        R_left = np.eye(3)
        R_right = R

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        
        stereo_width_px = stereo_height_px + self.max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + self.max_disp
        stereo_cy = (stereo_height_px - 1)/2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                        [0, stereo_focal_px, stereo_cy, 0],
                        [0,               0,         1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0]*stereo_focal_px

        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        Q = np.array([[1, 0,       0, -(stereo_cx - self.max_disp)],
                      [0, 1,       0, -stereo_cy],
                      [0, 0,       0, stereo_focal_px],
                      [0, 0, -1/T[0], 0]])

        # Create an undistortion map for the left and right camera which applies the
        # rectification and undoes the camera distortion. This only has to be done
        # once
        print("creating StereoRectify::UndistortionMap")
        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        self.undistort_rectify = {"left"  : (lm1, lm2),
                            "right" : (rm1, rm2)}
        self.img = None
        self.running = True

    def poll(self):
        try:
            self.frame_mutex.acquire()
            valid = self.frame_data["timestamp_ms"] is not None
            self.frame_mutex.release()
        except Exception as e:
            logging.error(e)
            return

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            self.frame_mutex.acquire()
            frame_copy = {"left"  : self.frame_data["left"].copy(),
                          "right" : self.frame_data["right"].copy()}
            self.frame_mutex.release()

            # Undistort and crop the center of the frames
            center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                          map1 = self.undistort_rectify["left"][0],
                                          map2 = self.undistort_rectify["left"][1],
                                          interpolation = cv2.INTER_LINEAR),
                                  "right" : cv2.remap(src = frame_copy["right"],
                                          map1 = self.undistort_rectify["right"][0],
                                          map2 = self.undistort_rectify["right"][1],
                                          interpolation = cv2.INTER_LINEAR)}
            # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
            disparity = self.stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0
            # re-crop just the valid part of the disparity
            disparity = disparity[:,self.max_disp:]
            # convert disparity to 0-255 and color it
            disp_vis = 255*(disparity - self.min_disp)/ self.num_disp

            img_l = center_undistorted["left"][:,self.max_disp:]
            img_r = center_undistorted["right"][:,self.max_disp:]

            if img_l is not None and img_r is not None:
                width, height = img_l.shape
		        # crop ROI to look at the road
                crop_height = height // 2
                grey_a = img_l[crop_height:height,0:width]
                grey_b = img_r[crop_height:height,0:width]
                grey_c = disp_vis[crop_height:height,0:width]
                
                stereo_image = np.zeros([crop_height, width, 3], dtype=np.dtype('B'))
                stereo_image[...,0] = np.reshape(grey_a, (crop_height, width))
                stereo_image[...,1] = np.reshape(grey_b, (crop_height, width))
                stereo_image[...,2] = np.reshape(grey_c, (crop_height, width))

                self.img = np.array(stereo_image)


    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.img

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


class RS_T265(object):
    '''
    The Intel Realsense T265 camera is a device which uses an imu, twin fisheye cameras,
    and an Movidius chip to do sensor fusion and emit a world space coordinate frame that 
    is remarkably consistent.
    '''

    def __init__(self, image_w=100, image_h=100, frame_rate=15, image_output=True, stereo=False, imu_output=False, motion=False):
        #Using the image_output will grab two image streams from the fisheye cameras but return only one.
        #This can be a bit much for USB2, but you can try it. Docs recommend USB3 connection for this.
        self.stereo = stereo
        self.image_output = image_output
        self.imu_output = imu_output
        self.motion = motion

        if self.motion:
            self.stereo = False
            self.image_output = False
            self.imu_output = False

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        
        if self.imu_output or self.motion:
            cfg.enable_stream(rs.stream.pose)

        if self.image_output:
            #right now it's required for both streams to be enabled
            cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
            cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.rot = zero_vec
        self.limg = None
        self.rimg = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.imu_output or self.motion:
            data = frames.get_pose_frame().get_pose_data()
            self.pos = data.translation
            self.vel = data.velocity
            self.acc = data.acceleration
            w = data.rotation.w
            x = -data.rotation.z
            y = data.rotation.x
            z = -data.rotation.y

            pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi
            self.rot = [roll, pitch, yaw]
        if self.image_output:
            self.limg = np.asanyarray(frames.get_fisheye_frame(1).get_data())
            if self.stereo:
                self.rimg = np.asanyarray(frames.get_fisheye_frame(2).get_data())

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        if self.motion:
            return self.vel.x, self.vel.y, self.vel.z, self.acc.x, self.acc.y, self.acc.z, self.rot[0], self.rot[1], self.rot[2]
        
        if self.stereo:
            if self.imu_output:
                return self.limg, self.rimg, self.acc.x, self.acc.y, self.acc.z, self.rot[0], self.rot[1], self.rot[2]
            elif self.image_output:
                return self.limg, self.rimg
        else:
            if self.imu_output:
                return self.limg, self.acc.x, self.acc.y, self.acc.z, self.rot[0], self.rot[1], self.rot[2]
            elif self.imu_output:
                return self.limg

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


class RS_D435i(object):
    '''
    The Intel Realsense D435i camera is a RGBD camera with imu
    '''

    def __init__(self, image_w=320, image_h=240, frame_rate=15, img_type='color'):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        self.img_type = img_type

        if self.img_type == 'color':
            cfg.enable_stream(rs.stream.color, image_w, image_h, rs.format.bgr8, frame_rate)
        elif self.img_type == 'depth':
            cfg.enable_stream(rs.stream.depth, image_w, image_h, rs.format.z16, frame_rate)
        else:
            raise Exception("img_type >", img_type, "< not supported.")

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        self.img = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.img_type is 'color':
            self.img = np.asanyarray(frames.get_color_frame().get_data())
        elif self.img_type is 'depth':
            self.img = np.asanyarray(frames.get_depth_frame().get_data())
        else:
            raise Exception("img_type >", self.img_type, "< not supported.")

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.img

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()

if __name__ == "__main__":
    iter = 0
    cam = RS_T265_StereoRectified()
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    while True:
        img = cam.run()
        cv2.imshow(WINDOW_TITLE,img)
        key = cv2.waitKey(1)

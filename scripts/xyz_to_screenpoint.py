#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from sensor_msgs.msg import CameraInfo
from deco_with_fetch.srv import PointStamped, PointStampedResponse
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel

class XYZToScreenPoint(object):
    def __init__(self):
        self.cameramodels = PinholeCameraModel()
        self.base_frame_id = "base_link"
        self.camera_frame_id = "head_camera_rgb_optical_frame"

        self.set_camera_param()
    
    def set_camera_param(self):
        camera_msg = CameraInfo()
        camera_msg.header.frame_id = self.camera_frame_id
        camera_msg.height = 480
        camera_msg.width = 640
        camera_msg.distortion_model = "plumb_bob"
        camera_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_msg.K = [554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0]
        camera_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_msg.P = [554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_msg.binning_x = 0
        camera_msg.binning_y = 0
        camera_msg.roi.x_offset = 0
        camera_msg.roi.y_offset = 0
        camera_msg.roi.height = 0
        camera_msg.roi.width = 0
        camera_msg.roi.do_rectify = False
        self.cameramodels.fromCameraInfo(camera_msg)
    
    def srv_cb(self, req):
        point = (req.point.x, req.point.y, req.point.z)
        u, v = self.cameramodels.project3dToPixel(point)
        rospy.logdebug("u, v : {}, {}".format(u, v))
        pub_msg = Point()
        pub_msg.x = u
        pub_msg.y = v
        pub_msg.z = 0
        
        return PointStampedResponse(pub_msg)
    
if __name__ == '__main__':
    rospy.init_node("xyz_to_screenpoint")
    xyz_to_screenpoint = XYZToScreenPoint()
    print("=== Launch XYZ To ScreenPoint ===")
    s = rospy.Service("xyz_to_screen_point", PointStamped, xyz_to_screenpoint.srv_cb)
    rospy.spin()

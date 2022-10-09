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
        self.camera_frame_id = ""

        self.is_camera_arrived = False

        rospy.Subscriber("~input/camera_info", CameraInfo, self.camera_info_cb)

    def camera_info_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.camera_frame_id = msg.header.frame_id
        self.is_camera_arrived = True

    def srv_cb(self, req):
        if self.is_camera_arrived:
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

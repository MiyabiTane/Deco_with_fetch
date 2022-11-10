#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import glob
import numpy as np
import cv2
import roslib.packages
from copy import deepcopy

from geometry_msgs.msg import Point
from deco_with_fetch.srv import DecoStatus, DecoStatusResponse

from cv_bridge import CvBridge

class CheckDecoStatus:
    def __init__(self):
        package_path = roslib.packages.get_pkg_dir('deco_with_fetch')
        self.share_dir_path = package_path + "/scripts/share/"
        self.images_dir_path = package_path + "/scripts/images/"

        self.bridge = CvBridge()

    def check_status_srv_cb(self, req):
        self.print_info(req)
        return DecoStatusResponse([])
        
    def print_info(self, req):
        print("- deco_count: {}".format(req.deco_count))
        print("- box_num: {}".format(req.box_num))
        print("- decos_rec_uv: {}".format(np.array(req.decos_rec_uv)))

if __name__ == '__main__':
    rospy.init_node("check_deco_status")
    check_deco_status = CheckDecoStatus()
    s = rospy.Service("check_deco_status", DecoStatus, check_deco_status.check_status_srv_cb)
    print("=== CHECK DECO STATUS SERVICE START ===")
    rospy.spin()

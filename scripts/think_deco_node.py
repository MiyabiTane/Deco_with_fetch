#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import glob
import roslib.packages
import cv2
import os

from think_deco import ThinkDecoration, think_with_trained_pix2pix, remove_dup_deco
from make_deco_imgs import MakeDecoImgs, print_decoration_info

from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from deco_with_fetch.srv import DecoImgs, DecoImgsResponse

from cv_bridge import CvBridge


class ThinkDecorationNode:
    def __init__(self):
        self.dir_path = roslib.packages.get_pkg_dir('deco_with_fetch') + "/scripts"
        self.bridge = CvBridge()
        self.called_count = 0
        self.init_param()
    
    def init_param(self):
        self.deco_imgs = []
        self.deco_masks = []
        self.input_img = None
        self.output_img = None
        self.output_arr = [(0, 0, 0, 0)]

    def deco_srv_cb(self, req):
        print("... thinking decoration ...")
        self.init_param()
        # for debug
        print_decoration_info(req.decos_img, req.decos_pos, req.decos_dims, req.decos_rec_uv, req.dimg_rect_pos,
                                req.bimg_lt_pos, req.bimg_rb_pos, req.head_angle, req.look_at_point, req.look_at_uv)
        self.input_img = self.bridge.imgmsg_to_cv2(req.back_img, desired_encoding="bgr8")
        if not os.path.isdir(self.dir_path + "/images/" + str(self.called_count)):
            os.makedirs(self.dir_path + "/images/" + str(self.called_count))
        cv2.imwrite(self.dir_path + "/images/" + str(self.called_count) + "/back_img.jpg", self.input_img)
        make_deco_imgs = MakeDecoImgs(req.decos_img, req.bimg_lt_pos, req.bimg_rb_pos, req.decos_dims, req.decos_rec_uv, self.called_count)
        make_deco_imgs.main()
        files = glob.glob(self.dir_path + "/images/" + str(self.called_count) + "/input*.jpg")
        files = sorted(files, key=lambda x: int(x[-5]))
        for fi in files:
            deco_img = cv2.imread(fi)
            self.deco_imgs.append(deco_img)
        files = glob.glob(self.dir_path + "/images/" + str(self.called_count) + "/mask*.jpg")
        files = sorted(files, key=lambda x: int(x[-5]))
        for fi in files:
            mask_img = cv2.imread(fi, 0)
            self.deco_masks.append(mask_img)
        # make decoration img
        self.output_img = think_with_trained_pix2pix(self.input_img, self.called_count)
        # Visualize (for debug)
        # cv2.imwrite(self.dir_path + "/share/input.png", self.input_img)
        # cv2.imwrite(self.dir_path + "/share/output.png", self.output_img)
        # think placement of decorations
        decorated_pos = remove_dup_deco(self.input_img, self.called_count)
        think_deco = ThinkDecoration(self.deco_imgs, self.deco_masks, self.input_img, self.output_img, decorated_pos, self.called_count)
        self.output_arr = think_deco.GA_calc()

        self.called_count += 1

        # return result
        # position.x -> center_x, position.y -> center_y, orientation.x -> width, orientation.y -> length
        result_msg = PoseArray()
        for x, y, w, l in self.output_arr:
            pose_msg = Pose()
            point_msg = Point()
            quater_msg = Quaternion()
            point_msg.x = x + w / 2
            point_msg.y = y + l / 2
            point_msg.z = 0
            quater_msg.x = w
            quater_msg.y = l
            pose_msg.position = point_msg
            pose_msg.orientation = quater_msg
            result_msg.poses.append(pose_msg)

        return DecoImgsResponse(result_msg)


if __name__ == '__main__':
    rospy.init_node("think_decoration")
    think_deco_node = ThinkDecorationNode()
    s = rospy.Service("think_deco", DecoImgs, think_deco_node.deco_srv_cb)
    print("=== THINK DECORATION SERVICE START===")
    rospy.spin()

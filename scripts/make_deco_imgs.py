#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

import cv2
import copy
import glob
import roslib.packages
import numpy as np
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel

DIR_PATH = roslib.packages.get_pkg_dir('deco_with_fetch') + "/scripts"

# req.decos_img, req.decos_pos, req.decos_dims, req.decos_rec_uv, req.dimg_rect_pos,
# req.bimg_lt_pos, req.bimg_rb_pos, req.head_angle, req.look_at_point, req.look_at_uv
class MakeDecoImgs(object):
    def __init__(self, decos_img, bimg_lt_pos, bimg_rb_pos, decos_dims, decos_rec_uv):
        bridge = CvBridge()
        decos_img = bridge.imgmsg_to_cv2(decos_img, desired_encoding="bgr8")
        cv2.imwrite(DIR_PATH + "/images/decos_img.jpg", decos_img)
        self.decos_img = decos_img
        self.decos_img = decos_img
        self.bimg_lt_pos = bimg_lt_pos
        self.bimg_rb_pos = bimg_rb_pos
        self.decos_dims = decos_dims
        self.decos_rec_uv = decos_rec_uv

    def reorder_point(self, xs, ys):
        points = []
        for x, y in zip(xs, ys):
            points.append((int(x), int(y)))
        points.sort(key=lambda x:x[0])
        left_lst = points[:2]
        right_lst = points[2:]
        left_lst.sort(key=lambda x:x[1])
        right_lst.sort(key=lambda x:x[1])
        lt, lb, rt, rb = left_lst[0], left_lst[1], right_lst[0], right_lst[1]
        return lt, lb, rt, rb

    def draw_line(self, img, xs, ys):
        lt, lb, rt, rb = self.reorder_point(xs, ys)
        draw_point = [lt, rt, rb, lb]
        for i in range(len(draw_point)):
            n_i = 0 if i + 1 == len(xs) else i + 1
            cv2.line(img, draw_point[i], draw_point[n_i], (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        return img

    def debug_draw_deco_rect(self, input_img, save_name):
        vis_decos_img = copy.deepcopy(input_img)
        for i in range(len(self.decos_dims)):
            vis_decos_img = self.draw_line(vis_decos_img, self.decos_rec_uv[i].xs, self.decos_rec_uv[i].ys)
        cv2.imwrite(save_name, vis_decos_img)

    def convert_decoimg(self, save_name, save_mask_name, deco_3d_dims, deco_rec_uv):
        lt, lb, rt, rb = self.reorder_point(deco_rec_uv.xs, deco_rec_uv.ys)
        height = int(deco_3d_dims.x)
        width = int(deco_3d_dims.y)
        pts1 = np.float32([lt, rt, lb, rb])
        pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        M_matrix = cv2.getPerspectiveTransform(pts1, pts2)
        img_trans = cv2.warpPerspective(self.decos_img, M_matrix, (width, height))
        # cv2.imwrite(save_name, img_trans)
        # resize
        # img_trans = cv2.imread(save_name)
        bimg_width = self.bimg_lt_pos.y - self.bimg_rb_pos.y
        bimg_height = self.bimg_lt_pos.z - self.bimg_rb_pos.z
        ratio_w = deco_3d_dims.y / bimg_width
        ratio_h = deco_3d_dims.x / bimg_height
        deco_w = int(640 * ratio_w)
        deco_h = int(480 * ratio_h)
        deco_img = cv2.resize(img_trans, dsize=(deco_w, deco_h))
        cv2.imwrite(save_name, deco_img)
        # make mask img
        img_white = np.ones((deco_h, deco_w), np.uint8) * 255
        cv2.imwrite(save_mask_name, img_white)

    def main(self):
        self.debug_draw_deco_rect(self.decos_img, DIR_PATH + "/images/debug_decos_img.jpg")
        for i in range(len(self.decos_dims)):
            save_name = DIR_PATH + "/images/input" + str(i) + ".jpg"
            save_mask_name = DIR_PATH + "/images/mask" + str(i) + ".jpg"
            self.convert_decoimg(save_name, save_mask_name, self.decos_dims[i], self.decos_rec_uv[i])

def print_decoration_info(decos_img, decos_pos, decos_dims, decos_rec_uv, dimg_rect_pos,
                            bimg_lt_pos, bimg_rb_pos, head_angle, look_at_point, look_at_uv):
    # deco_bboxes's frame_id -> "head_mount_kinect_rgb_optical_frame"
    # other's frame_id -> "base_footprint"
    bridge = CvBridge()
    decos_jpg = bridge.imgmsg_to_cv2(decos_img, desired_encoding="bgr8")
    cv2.imwrite(DIR_PATH + "/images/decos_img.jpg", decos_jpg)
    print("Back Img info->")
    print("lt: ", bimg_lt_pos.x, bimg_lt_pos.y, bimg_lt_pos.z)
    print("rb: ", bimg_rb_pos.x, bimg_rb_pos.y, bimg_rb_pos.z)
    print("head_angle: ", head_angle.data)
    print("look_at_point: ", look_at_point)
    print("look_at_uv: ", look_at_uv)
    print("dimg_rect_pos: ", np.array(dimg_rect_pos))
    print("= decos_pos")
    print(np.array(decos_pos))
    print("= decos_dims")
    print(np.array(decos_dims))
    print("= decos_rec_uv")
    print(np.array(decos_rec_uv))

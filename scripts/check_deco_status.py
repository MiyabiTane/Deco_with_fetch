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

NOT_FOUND_TH = 0.5


class CheckDecoStatus:
    def __init__(self):
        package_path = roslib.packages.get_pkg_dir('deco_with_fetch')
        self.share_dir_path = package_path + "/scripts/share/"
        self.images_dir_path = package_path + "/scripts/images/"

        self.bridge = CvBridge()
    
    def debug_visualize(self, ga_back_img, res_img, count):
        blended_img = cv2.addWeighted(src1=ga_back_img, alpha=0.6, src2=res_img, beta=0.4, gamma=0)
        cv2.imwrite(self.images_dir_path + str(count) + "/debug.jpg", blended_img)
    
    def debug_visualize_line(self, img, cx, cy, deco_img):
        output_debug_img = deepcopy(img)
        h, w, _ = deco_img.shape
        lx, ly = int(cx - w / 2.0), int(cy - h / 2.0)
        rx, ry = int(cx + w / 2.0), int(cy + h / 2.0)
        cv2.rectangle(output_debug_img, (lx, ly), (rx, ry), (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.imwrite(self.share_dir_path + "debug.jpg", output_debug_img)

    def get_match_pos(self, back_img, deco_img, thresh):
        res = cv2.matchTemplate(back_img, deco_img, cv2.TM_CCOEFF_NORMED)
        _min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(res)
        if max_val > thresh:
            H, W, _ = deco_img.shape
            lx, ly = max_loc
            cx = int(lx + W / 2.0)
            cy = int(ly + H / 2.0)
            # self.debug_visualize_line(back_img, cx, cy, deco_img)
            return cx, cy
        return -1, -1

    def check_status_srv_cb(self, req):
        thresh = 1.1
        ga_back_img = cv2.imread(self.share_dir_path + "ga_output_" + str(req.deco_count) + ".jpg")
        real_back_img = self.bridge.imgmsg_to_cv2(req.back_img, desired_encoding="bgr8")
        # ga_back_img = cv2.imread(self.share_dir_path + "ga_output_status.jpg")
        # real_back_img = cv2.imread(self.share_dir_path + "result_0.jpg")
        deco_img = cv2.imread(self.images_dir_path + str(req.deco_count) + "/input" + str(req.box_num) + ".jpg")
        ga_x, ga_y, res_x, res_y = -1, -1, -1, -1
        while -1 in [ga_x, ga_y, res_x, res_y] and thresh >= NOT_FOUND_TH:
            thresh -= 0.1
            print("thresh: ", thresh)
            ga_x, ga_y = self.get_match_pos(ga_back_img, deco_img, thresh)
            res_x, res_y = self.get_match_pos(real_back_img, deco_img, thresh)
        # print("== thresh: ", thresh)
        if thresh < NOT_FOUND_TH:
            # 飾り付けに失敗
            return DecoStatusResponse([])
        # 今ロボットが飾り付けた画像を基準に他の飾りのずれを見る
        afin_matrix = np.float32([[1, 0, ga_x - res_x], [0, 1, ga_y - res_y]])
        res_img = cv2.warpAffine(real_back_img, afin_matrix, (640, 480))
        self.debug_visualize(ga_back_img, res_img, req.deco_count)

        result_lst = []
        decos_files = glob.glob(self.images_dir_path + str(req.deco_count) + "/input*.jpg")
        for i in range(len(decos_files)):
            fi = self.images_dir_path + str(req.deco_count) + "/input" + str(i) + ".jpg"
            deco_img = cv2.imread(fi)
            ga_x, ga_y = self.get_match_pos(ga_back_img, deco_img, thresh)
            res_x, res_y = self.get_match_pos(res_img, deco_img, thresh)
            print("({}, {}) -> ({}, {})".format(ga_x, ga_y, res_x, res_y))
            point_msg = Point()
            point_msg.x = res_x
            point_msg.y = res_y
            result_lst.append(point_msg)
        
        return DecoStatusResponse(result_lst)

if __name__ == '__main__':
    rospy.init_node("check_deco_status")
    check_deco_status = CheckDecoStatus()
    s = rospy.Service("check_deco_status", DecoStatus, check_deco_status.check_status_srv_cb)
    print("=== CHECK DECO STATUS SERVICE START ===")
    rospy.spin()

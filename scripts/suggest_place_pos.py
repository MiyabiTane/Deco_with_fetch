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

package_path = roslib.packages.get_pkg_dir('deco_with_fetch')
DISTANCE_TH = 50

class SuggestPlacePos:
    def __init__(self):
        self.bridge = CvBridge()
    
    def init_param(self):
        self.images_dir_path = ""
        self.input_imgs = []
    
    def set_input_imgs(self, deco_count):
        input_files = glob.glob(self.images_dir_path + "input*.jpg")
        for i in range(len(input_files)):
            fi = self.images_dir_path + "input" + str(i) + ".jpg"
            input_img = cv2.imread(fi)
            self.input_imgs.append(input_img)
    
    def suggest_pos_srv_cb(self, req):
        self.print_info(req)
        self.init_param()
        self.images_dir_path = package_path + "/scripts/images/" + str(req.deco_count) + "/"
        self.set_input_imgs(req.deco_count)
        back_img = self.bridge.imgmsg_to_cv2(req.back_img, desired_encoding="bgr8")
        try:
            found_deco_img = self.clip_deco_img(req.decos_rec_uv, back_img)
            match_num = self.get_similar_img_num(found_deco_img)
        except:
            return DecoStatusResponse([])
        if match_num == -1:
            return DecoStatusResponse([])
        point_msg = Point()
        point_msg.z = match_num
        return DecoStatusResponse([point_msg])
    
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
    
    def debug_draw_line(self, img, lt, lb, rt, rb, color=(255, 0, 0)):
        debug_img = deepcopy(img)
        draw_point = [lt, rt, rb, lb]
        for i in range(len(draw_point)):
            n_i = 0 if i + 1 == len(draw_point) else i + 1
            cv2.line(debug_img, draw_point[i], draw_point[n_i], color, thickness=2, lineType=cv2.LINE_4)
        return debug_img
    
    def clip_deco_img(self, decos_rec_uv, back_img):
        debug_img = deepcopy(back_img)
        lt, lb, rt, rb = self.reorder_point(decos_rec_uv[0].xs, decos_rec_uv[0].ys)
        debug_img = self.debug_draw_line(debug_img, lt, lb, rt, rb)
        min_ypos = min(max(0, min(lt[1], rt[1])), 480)
        max_ypos = min(max(0, max(lb[1], rb[1])), 480)
        min_xpos = min(max(0, min(lt[0], lb[0])), 640)
        max_xpos = min(max(0, max(rt[0], rb[0])), 640)
        found_deco_img = back_img[min_ypos: max_ypos, min_xpos: max_xpos]
        cv2.imwrite(self.images_dir_path + "debug_asked_deco.jpg", debug_img)
        cv2.imwrite(self.images_dir_path + "asked_deco.jpg", found_deco_img)
        return found_deco_img
    
    def get_similar_img_num(self, target_img):
        
        def get_average_color(img):
            h, w, c = img.shape
            min_ypos = min(max(0, h/2 - 25), 480)
            max_ypos = min(max(0, h/2 + 25), 480)
            min_xpos = min(max(0, w/2 - 25), 640)
            max_xpos = min(max(0, w/2 + 25), 640)
            img = img[min_ypos: max_ypos, min_xpos: max_xpos]
            h, w, c = img.shape
            color_arr = img.reshape(h * w, c)
            color_mean = np.mean(color_arr, axis=0)
            color_mean = np.array(color_mean) * 382 / np.sum(color_mean)
            color_mean = color_mean.astype(int)
            return color_mean
        
        for i, input_img in enumerate(self.input_imgs):
            ans_num = -1
            min_distance = DISTANCE_TH
            target_color = get_average_color(target_img)
            compare_color = get_average_color(input_img)
            distance = np.linalg.norm(np.array(target_color) - np.array(compare_color))
            if distance < min_distance:
                ans_num = i
                min_distance = distance
        print("matched_num: {}".format(ans_num))
        return ans_num

    def print_info(self, req):
        print("- deco_count: {}".format(req.deco_count))
        print("- box_num: {}".format(req.box_num))
        print("- decos_rec_uv: {}".format(np.array(req.decos_rec_uv)))

if __name__ == '__main__':
    rospy.init_node("suggest_place_pos")
    suggest_place_pos = SuggestPlacePos()
    s = rospy.Service("suggest_place_pos", DecoStatus, suggest_place_pos.suggest_pos_srv_cb)
    print("=== SUGGEST PLACE POS SERVICE START ===")
    rospy.spin()

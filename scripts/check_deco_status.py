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
HIST_TH = 0.1

class CheckDecoStatus:
    def __init__(self):
        self.share_dir_path = package_path + "/scripts/share/"
        self.images_dir_path = ""

        self.bridge = CvBridge()
        self.result_img = None
        self.ideal_ga_img = None
        self.decorated_imgs = []
        self.input_imgs = []
        self.matched_box_num = -1

    def set_input_imgs(self, deco_count):
        input_files = glob.glob(self.images_dir_path + "input*.jpg")
        for i in range(len(input_files)):
            fi = self.images_dir_path + "input" + str(i) + ".jpg"
            input_img = cv2.imread(fi)
            self.input_imgs.append(input_img)

    def check_status_srv_cb(self, req):
        self.images_dir_path = package_path + "/scripts/images/" + str(req.deco_count) + "/"
        self.result_img = self.bridge.imgmsg_to_cv2(req.back_img, desired_encoding="bgr8")
        # self.result_img = cv2.imread(self.images_dir_path + "result_fake.jpg")
        cv2.imwrite(self.images_dir_path + "result.jpg", self.result_img)
        self.ideal_ga_img = cv2.imread(self.share_dir_path + "ga_output_" + str(req.deco_count) + ".jpg")
        self.set_input_imgs(req.deco_count)
        self.print_info(req)

        self.clip_found_decoration(req.decos_rec_uv)
        # 今ロボットが飾り付けた画像を基準に他の飾りのずれを見る
        match_num = self.get_similar_img_num(self.input_imgs[req.box_num], self.decorated_imgs)
        if match_num == -1:
            return DecoStatusResponse([])
        self.matched_box_num = match_num
        ga_pos_x, ga_pos_y = int(req.ga_calc_uv[req.box_num].x), int(req.ga_calc_uv[req.box_num].y)
        real_pos_x = int(sum(req.decos_rec_uv[match_num].xs) / 4.0)
        real_pos_y = int(sum(req.decos_rec_uv[match_num].ys) / 4.0)
        diff_x, diff_y = real_pos_x - ga_pos_x, real_pos_y - ga_pos_y

        ans_lst = [(-1, -1)] * len(self.decorated_imgs)
        for i in range(len(self.input_imgs)):
            match_num = self.get_similar_img_num(self.input_imgs[i], self.decorated_imgs)
            if match_num != -1:
                ideal_x = req.ga_calc_uv[i].x + diff_x
                ideal_y = req.ga_calc_uv[i].y + diff_y
                ans_lst[match_num] = (ideal_x, ideal_y)
        self.debug_view(diff_x, diff_y, req.ga_calc_uv, req.decos_rec_uv, ans_lst)

        res_lst = []
        for i, ans in enumerate(ans_lst):
            point_msg = Point()
            point_msg.x = ans[0]
            point_msg.y = ans[1]
            if i == self.matched_box_num:
                point_msg.z = 1  # 今ロボットが置いた飾り
            res_lst.append(point_msg)
        return DecoStatusResponse(res_lst)

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

    def clip_found_decoration(self, decos_rec_uv):
        debug_decorated_img = deepcopy(self.result_img)
        for i in range(len(decos_rec_uv)):
            lt, lb, rt, rb = self.reorder_point(decos_rec_uv[i].xs, decos_rec_uv[i].ys)
            debug_decorated_img = self.debug_draw_line(debug_decorated_img, lt, lb, rt, rb)
            decorated_img = self.result_img[min(lt[1], rt[1]): max(lb[1], rb[1]), min(lt[0], lb[0]): max(rt[0], rb[0])]
            try:
                cv2.imwrite(self.images_dir_path + "decorated" + str(i) + ".jpg", decorated_img)
            except:
                print("imwrite error")
            self.decorated_imgs.append(decorated_img)
        cv2.imwrite(self.images_dir_path + "debug_decorated.jpg", debug_decorated_img)

    def get_similar_img_num(self, target_img, imgs):
        # use histgram of color
        print("")
        ans_num = -1
        max_sim_point = HIST_TH
        h, w, _ = target_img.shape
        target_hist = cv2.calcHist([target_img], [0], None, [256], [0, 256])
        for i, img in enumerate(imgs):
            img = cv2.resize(img, (w, h))
            compare_hist = cv2.calcHist([img], [0], None, [256], [0, 256])
            ret = cv2.compareHist(target_hist, compare_hist, 0)
            print("{}: {}".format(i, ret))
            if ret > max_sim_point:
                ans_num = i
                max_sim_point = ret
        print("matched_num: {}".format(ans_num))
        return ans_num

    def debug_view(self, diff_x, diff_y, ga_calc_uv, decos_rec_uv, ans_lst):
        ideal_img = deepcopy(self.ideal_ga_img)
        for i in range(len(ga_calc_uv)):
            ga_x, ga_y = ga_calc_uv[i].x, ga_calc_uv[i].y
            h, w, _c = self.input_imgs[i].shape
            lt = (int(ga_x - w/2.0), int(ga_y - h/2.0))
            lb = (int(ga_x - w/2.0), int(ga_y + h/2.0))
            rt = (int(ga_x + w/2.0), int(ga_y - h/2.0))
            rb = (int(ga_x + w/2.0), int(ga_y + h/2.0))
            ideal_img = self.debug_draw_line(ideal_img, lt, lb, rt, rb, color=(0, 255, 0))
        result_img = cv2.imread(self.images_dir_path + "debug_decorated.jpg")
        affin_matrix = np.float32([[1, 0, diff_x], [0, 1, diff_y]])
        ideal_img = cv2.warpAffine(ideal_img, affin_matrix, (640, 480))
        blended_img = cv2.addWeighted(src1=result_img, alpha=0.6, src2=ideal_img, beta=0.4, gamma=0)
        for i, ans in enumerate(ans_lst):
            if int(ans[0]) != -1:
                pt1 = (int(sum(decos_rec_uv[i].xs) / 4.0), int(sum(decos_rec_uv[i].ys) / 4.0))
                pt2 = (int(ans[0]), int(ans[1]))
                print("decorated_{}: {} -> {}".format(i, pt1, pt2))
                cv2.arrowedLine(blended_img, pt1, pt2, (0, 0, 255), thickness=2, tipLength=0.3)
        cv2.imwrite(self.images_dir_path + "deco_status.jpg", blended_img)
        
    def print_info(self, req):
        print("- deco_count: {}".format(req.deco_count))
        print("- box_num: {}".format(req.box_num))
        print("- decos_rec_uv: {}".format(np.array(req.decos_rec_uv)))
        print("- ga_calc_uv: {}".format(np.array(req.ga_calc_uv)))

if __name__ == '__main__':
    rospy.init_node("check_deco_status")
    check_deco_status = CheckDecoStatus()
    s = rospy.Service("check_deco_status", DecoStatus, check_deco_status.check_status_srv_cb)
    print("=== CHECK DECO STATUS SERVICE START ===")
    rospy.spin()

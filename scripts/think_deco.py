#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import glob
import roslib.packages
import numpy as np
import subprocess
import random
from copy import deepcopy
from collections import deque
from scipy.spatial import distance

ALPHA = 0
DIFF_TH = 50
SIZE_TH = 1000
INPUT_NAME = "share/pix2pix_input.jpg"
OUTPUT_PATH = "share/pix2pix_output"
DIR_PATH = roslib.packages.get_pkg_dir('deco_with_fetch') + "/scripts/"


# you need to run: deco_with_fetch/scripts$ docker image build -t deco_tensor .
def think_with_trained_pix2pix(input_img, called_count):
    # input_img.shape = (480, 640, 3)
    img = np.full((640, 640, 3), 255)
    img[90: 570] = input_img
    cv2.imwrite(DIR_PATH + INPUT_NAME, img)
    # 学習済みpix2pixによる飾り付け画像生成
    output_name = OUTPUT_PATH + "_" + str(called_count) + ".jpg"
    subprocess.call(["docker", "run", "--rm", "-it", "--mount", "type=bind,source=" + DIR_PATH + "share,target=/deco_tensor/share",
                    "deco_tensor", "python3", "trained_pix2pix.py", "--input", INPUT_NAME, "--output", output_name])
    output_img = cv2.imread(DIR_PATH + output_name)
    output_img = cv2.resize(output_img , (640, 640))
    output_img = output_img[90: 570]
    return output_img


def remove_dup_deco(init_img, back_img, called_count):
    def check_dup(d_lx, d_ly, d_rx, d_ry, decorated_pos):
        for lx, ly, rx, ry in decorated_pos:
            y_len = min(ry, d_ry) - max(ly, d_ly)
            x_len = min(rx, d_rx) - max(lx, d_lx)
            if y_len > 0 and x_len > 0:
                dup_area =  y_len * x_len
                if dup_area / ((ry - ly) * (rx - lx)) > 0.7:
                    return True
        return False

    def get_diff_img(before_img, after_img, k_size=5, k_size2=5):
        im_diff = before_img.astype(int) - after_img.astype(int)
        im_diff_abs = np.abs(im_diff)
        im_diff_img = im_diff_abs.astype(np.uint8)
        im_diff_img[np.where(im_diff_abs[:,:,0] < DIFF_TH) and np.where(im_diff_abs[:,:,1] < DIFF_TH) and np.where(im_diff_abs[:,:,2] < DIFF_TH)] = [0, 0, 0]
        img_gray = cv2.cvtColor(im_diff_img, cv2.COLOR_BGR2GRAY)
        _, img_binary = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
        # remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k_size,k_size))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT,(k_size2,k_size2))
        img_opening = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, kernel)
        img_closing = cv2.morphologyEx(img_opening, cv2.MORPH_CLOSE, kernel2)
        return img_closing

    # 2回目以降の飾り付け生成では既に飾りがあるところに置かないようにする
    decorated_pos = []
    diff_img = get_diff_img(init_img, back_img)
    contours, _hierarchy = cv2.findContours(diff_img ,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > SIZE_TH:
            if cv2.contourArea(cnt) < 480 * 640 / 3.0:
                lx, ly, w, h = cv2.boundingRect(cnt)
                rx, ry = lx + w, ly + h
                if not check_dup(lx, ly, rx, ry, decorated_pos):
                    decorated_pos.append((int(lx), int(ly), int(rx), int(ry)))
    # debug
    for lx, ly, rx, ry in decorated_pos:
        cv2.rectangle(diff_img, (lx, ly), (rx, ry), (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
    cv2.imwrite(DIR_PATH + "images/" + str(called_count) + "/decorated_pos.jpg", diff_img)

    return decorated_pos


class ThinkDecoration:
    def __init__(self, deco_imgs, deco_masks, input_img, output_img, cannot_place_pos, called_count, bimg_rb_uv, nums=21, generation=30, elite=2):
        # 複数の飾りが重ならないようにする 0: 空きスペース, 1: 飾りが既にある, 2: 飾りが既にあるor壁がない、書き換え不可能
        self.visited = np.zeros((480, 640), dtype=np.int)
        for lx, ly, rx, ry in cannot_place_pos:
            self.visited[ly: ry, lx: rx] = 2
        print("Cannot place pos: ", cannot_place_pos)
        # print(np.array(self.visited))
        # print(np.where(np.array(self.visited)==0))
        self.input = input_img
        self.H, self.W, _ = self.input.shape
        self.output = output_img
        self.imgs = deco_imgs
        self.masks = deco_masks
        self.nums = nums
        self.elite = elite
        self.generation = generation
        # 貼り付け位置
        self.genes = []
        self.best_gene = (-1 * float('inf'), None)
        self.called_count = called_count
        self.bimg_rb_uv = bimg_rb_uv

        for _ in range(nums):
            gene = []
            for im in self.imgs:
                h, w, _ = im.shape
                random_x = random.randint(w / 2, self.W - w / 2)
                random_y = random.randint(h / 2, self.H - h / 2)
                # print(h, w, random_x, random_y)
                gene.append((random_x, random_y, h, w))
            gene = self.remove_overlap(gene)
            self.genes.append(gene)
            output_img = self.generate_img(gene)
        # print(self.genes)


    def shift_pos(self, pos_x, pos_y, h, w):
        if pos_x - w / 2 < 0:
            pos_x = int(w / 2)
        elif pos_x + w / 2 > self.W:
            pos_x = int(self.W - w / 2)
        if pos_y - h / 2 < 0:
            pos_y = int(h / 2)
        elif pos_y + h / 2 >= self.H:
            pos_y = int(self.H - h / 2)
        return pos_x, pos_y


    def remove_overlap(self, gene, debug=False):
        new_gene = []
        self.visited = np.where(self.visited == 1, 0, self.visited)
        for pos_x, pos_y, h, w in gene:
            new_x, new_y = self.generate_new_pos(pos_x, pos_y, h, w, debug)
            self.visited[int(new_y - h/2.0): int(new_y + h/2.0), int(new_x - w/2.0): int(new_x + w/2.0)] = 1
            new_gene.append((new_x, new_y, h, w))
        return new_gene


    def generate_new_pos(self, pos_x, pos_y, h, w, debug=False):
        to_visit = deque([(pos_x, pos_y)])
        count = 0
        while to_visit and count < 100:
            count += 1
            pos_x, pos_y = to_visit.popleft()
            check_array = self.visited[int(pos_y - h/2.0): int(pos_y + h/2.0), int(pos_x - w/2.0): int(pos_x + w/2.0)]
            over_y, over_x = np.where((check_array == 1) | (check_array == 2))
            over_y, over_x = set(over_y), set(over_x)
            if debug:
                print(over_y, over_x)
            if len(over_x) == 0 and len(over_y) == 0:
                if debug:
                    print("No overlap ", count)
                return pos_x, pos_y
            # 右側にずらした時の座標
            ans_x = pos_x + max(over_x) + 1
            if ans_x + w / 2 <= self.W:
                to_visit.append((ans_x, pos_y))
            # 左側にずらした時の座標
            ans_x = pos_x - (w - min(over_x))
            if ans_x - w / 2 >= 0:
                to_visit.append((ans_x, pos_y))
            # 下側側にずらした時の座標
            ans_y = pos_y + max(over_y) + 1
            if ans_y + h / 2 <= self.H:
                to_visit.append((pos_x, ans_y))
            # 上側にずらした時の座標
            ans_y = pos_y - (h - min(over_y))
            if ans_y - h / 2 >= 0:
                to_visit.append((pos_x, ans_y))
        # どこへもずらせない時は画像の右下に配置
        return int((self.W - self.bimg_rb_uv.x) - w / 2), int((self.H - self.bimg_rb_uv.y) - h / 2)


    def generate_img(self, gene):
        # self.visited = np.where(self.visited == 1, 0, self.visited))
        output_img = deepcopy(self.input)
        for i, deco_img in enumerate(self.imgs):
            pos_x, pos_y, h, w = gene[i]
            mask_img = self.masks[i]
            min_ypos = min(max(0, int(pos_y - h/2.0)), 480)
            max_ypos = min(max(0, int(pos_y + h/2.0)), 480)
            min_xpos = min(max(0, int(pos_x - w/2.0)), 640)
            max_xpos = min(max(0, int(pos_x + w/2.0)), 640)
            back_img = output_img[min_ypos: max_ypos, min_xpos: max_xpos]
            # print(back_img.shape, deco_img.shape, mask_img.shape)
            try:
                deco_img[mask_img < 150] = [0, 0, 0]
                back_img[mask_img >= 150] = [0, 0, 0]
            except:
                deco_img = cv2.resize(deco_img, dsize=(int(pos_y + h/2.0) - int(pos_y - h/2.0), int(pos_x + w/2.0) - int(pos_x - w/2.0)))
                mask_img = cv2.resize(mask_img, dsize=(int(pos_y + h/2.0) - int(pos_y - h/2.0), int(pos_x + w/2.0) - int(pos_x - w/2.0)))
                back_img = cv2.resize(back_img, dsize=(int(pos_y + h/2.0) - int(pos_y - h/2.0), int(pos_x + w/2.0) - int(pos_x - w/2.0)))
                deco_img[mask_img < 150] = [0, 0, 0]
                back_img[mask_img >= 150] = [0, 0, 0]
            comp_img = cv2.add(deco_img, back_img)
            try:
                output_img[int(pos_y - h/2.0): int(pos_y + h/2.0), int(pos_x - w/2.0): int(pos_x + w/2.0)] = comp_img
            except:
                comp_h, comp_w, _ = comp_img.shape
                output_img[int(pos_y - h/2.0): int(pos_y - h/2.0) + comp_h, int(pos_x - w/2.0): int(pos_x - w/2.0) + comp_w] = comp_img
        # cv2.imwrite("output0707.jpg", output_img)
        return output_img


    def calc_img_sim(self, img1, img2):
        # 画像のpHashのハミング距離を取る
        # 類似度が大きければ1, 小さければ0

        def hash_array_to_hash_hex(hash_array):
            # convert hash array of 0 or 1 to hash string in hex
            hash_array = np.array(hash_array, dtype = np.uint8)
            hash_str = ''.join(str(i) for i in 1 * hash_array.flatten())
            return (hex(int(hash_str, 2)))

        def hash_hex_to_hash_array(hash_hex):
            # convert hash string in hex to hash values of 0 or 1
            hash_str = int(hash_hex, 16)
            array_str = bin(hash_str)[2:]
            return np.array([i for i in array_str], dtype = np.float32)

        def calc_hash(img):
            # resize image and convert to gray scale
            img = cv2.resize(img, (64, 64))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = np.array(img, dtype = np.float32)
            # calculate dct of image
            dct = cv2.dct(img)
            # to reduce hash length take only 8*8 top-left block
            # as this block has more information than the rest
            dct_block = dct[: 8, : 8]
            # caclulate mean of dct block excluding first term i.e, dct(0, 0)
            dct_average = (dct_block.mean() * dct_block.size - dct_block[0, 0]) / (dct_block.size - 1)
            # convert dct block to binary values based on dct_average
            dct_block[dct_block < dct_average] = 0.0
            dct_block[dct_block != 0] = 1.0
            return hash_array_to_hash_hex(dct_block.flatten())

        hash1 = calc_hash(img1)
        hash2 = calc_hash(img2)
        dis = distance.hamming(list(hash1), list(hash2))
        return 1.0 - dis


    def evaluate(self):
        points = []
        for i in range(self.nums):
            decorated_img = self.generate_img(self.genes[i])
            # 各飾りに関して、置く前と置いた後のどちらが類似度が高いかを調べる
            point = 0
            for pos_x, pos_y, h, w in self.genes[i]:
                ly, ry, lx, rx = int(pos_y - h/2.0), int(pos_y + h/2.0), int(pos_x - w/2.0), int(pos_x + w/2.0)
                similarity = self.calc_img_sim(decorated_img[ly: ry, lx: rx, :], self.output[ly: ry, lx: rx, :])
                point += similarity
            points.append(point)
        points = np.array(points)
        print(max(points))
        if self.best_gene[0] < max(points):
            best_index = np.argsort(points)[-1]
            self.best_gene = (points[best_index], self.genes[best_index])
            # print(self.best_gene)
        points = map(lambda x: x-(min(points)), points)
        if float(sum(points)) == 0:
            points = [1.0/len(points)] * len(points)
        else:
            points = map(lambda x: float(x)/float(sum(points)), points)
        return points


    def partial_crossover(self, parent_1, parent_2):
        num = len(parent_1)
        cross_point = random.randrange(2, num-1) if num > 3 else 0
        child_1 = parent_1
        child_2 = parent_2
        for i in range(num - cross_point):
            target_index = cross_point + i
            x1, y1, h, w = parent_1[target_index]
            x2, y2, _h, _w = parent_2[target_index]
            min_x, max_x = min(x1, x2), max(x1, x2)
            min_y, max_y = min(y1, y2), max(y1, y2)
            min_nx = int(min_x - ALPHA * random.random() * (max_x - min_x))
            max_nx = int(max_x + ALPHA * random.random() * (max_x - min_x))
            min_ny = int(min_y - ALPHA * random.random() * (max_y - min_y))
            max_ny = int(max_y + ALPHA * random.random() * (max_y - min_y))
            p1_x = random.randint(min_nx, max_nx)
            p2_x = random.randint(min_nx, max_nx)
            p1_y = random.randint(min_ny, max_ny)
            p2_y = random.randint(min_ny, max_ny)
            p1_x, p1_y = self.shift_pos(p1_x, p1_y, h, w)
            p2_x, p2_y = self.shift_pos(p2_x, p2_y, h, w)
            child_1[target_index] = (p1_x, p1_y, h, w)
            child_2[target_index] = (p2_x, p2_y, h, w)
        child_1 = self.remove_overlap(child_1)
        child_2 = self.remove_overlap(child_2)
        return child_1, child_2
        # ブレンド交叉
        # https://qiita.com/simanezumi1989/items/4f821de2b77850fcf508


    def generate_next_generation(self):
        points = self.evaluate()
        copy = deepcopy(self.genes)
        for i in range((self.nums - self.elite)//2):
            index_1, index_2 = np.random.choice(len(points), 2, replace = True, p = points)
            #print(index_1, index_2)
            parent_1 = self.genes[index_1]
            parent_2 = self.genes[index_2]
            child_1, child_2 = self.partial_crossover(parent_1, parent_2)
            copy[2*i] = child_1
            copy[2*i + 1] = child_2
        # inherit high point parents
        elite_parent_index = np.argsort(points)
        for i in range(self.nums - self.elite, self.nums):
            copy[i] = self.genes[elite_parent_index[i]]
        self.genes = deepcopy(copy)


    def mutation(self, num_mutation = 3, mute_per = 0.7):
    # num_mutaiton can mutate at the percentage of mute_per 
        mutation_genes = np.random.choice(len(self.genes), num_mutation, replace = False)
        for index in mutation_genes:
            flag = np.random.choice(2, 1, p = [1 - mute_per, mute_per])
            if flag == 1:
                m_index = np.random.choice(len(self.imgs), 1, replace = False)[0]
                h, w, _ = self.imgs[m_index].shape
                random_x = random.randint(int(w / 2.0), int(self.W - w / 2.0)) if self.W > w / 2.0 + 1 else 0
                random_y = random.randint(int(h / 2.0), int(self.H - h / 2.0)) if self.H > h / 2.0 + 1 else 0
                self.genes[index][m_index] == (random_x, random_y, h, w)
                self.genes[index] = self.remove_overlap(self.genes[index])


    def GA_calc(self):
        for _ in range(self.generation):
            self.generate_next_generation()
            self.mutation()
            # print(self.genes)
        best_point, best_gene = self.best_gene
        print("BEST POINT: ", best_point)
        best_gene = self.remove_overlap(best_gene, debug=False)
        print(best_gene)
        output_img = self.generate_img(best_gene)
        cv2.imwrite(DIR_PATH + "share/ga_output_" + str(self.called_count) + ".jpg", output_img)
        return best_gene


# GA http://samuiui.com/2019/10/27/python%E3%81%A7%E9%81%BA%E4%BC%9D%E7%9A%84%E3%82%A2%E3%83%AB%E3%82%B4%E3%83%AA%E3%82%BA%E3%83%A0%EF%BC%88ga%EF%BC%89%E3%82%92%E5%AE%9F%E8%A3%85%E3%81%97%E3%81%A6%E5%B7%A1%E5%9B%9E%E3%82%BB%E3%83%BC/


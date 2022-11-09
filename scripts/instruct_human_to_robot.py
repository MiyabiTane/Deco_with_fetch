#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal

class InstructChat(object):
    def __init__(self):
        self.syntaxes = []
        self.lemma_lst = []
        self.init_param()
        self.set_words_lst()
        self.called_count = 0
       
        self.actionlib_client = actionlib.SimpleActionClient("/analyze_text/text", AnalyzeTextAction)
        self.actionlib_client.wait_for_server()
    
    def init_param(self):
        self.move_up = False
        self.move_down = False
        self.move_left = False
        self.move_right = False
        self.degree = -1  # 0: 少し, 1: たくさん

        self.called_count = -1

    def set_words_lst(self):
        self.clear_direction = ["左", "右", "上", "下"]
        self.indicator_direction = ["そっち", "こっち", "あっち", "ここ", "あそこ"]
        self.indicator_here = ["そこ"]
        self.place = ["場所", "所"]
        self.request_words = ["欲しい", "方が", "が良い"]
        self.degree_words = ["もっと", "たくさん", "少し", "ちょっと"]
        self.excess_and_denial = ["過ぎ", "逆", "違う", "戻し", "じゃない"]
        self.agree_words = ["うん", "はい", "良い"]
        self.action_verb = ["置い", "動かす", "動かし", "ずらし"]

        self.trigger_words = self.indicator_direction + self.indicator_here + self.place
    
    def convert_sentence(self, sentence):
        sentence = sentence.replace("よい", "良い")
        sentence = sentence.replace("いい", "良い")
        sentence = sentence.replace("ほう", "方")
        sentence = sentence.replace("すぎ", "過ぎ")
        sentence = sentence.replace("ちがう", "違う")
        sentence = sentence.replace("もどし", "戻し")
        sentence = sentence.replace("ほしい", "欲しい")
        sentence = sentence.replace("ばしょ", "場所")
        sentence = sentence.replace("ところ", "所")
        return sentence
    
    def get_parsing_result(self, input_sentence):
        pub_action_msg = AnalyzeTextGoal()
        pub_action_msg.text = input_sentence
        self.actionlib_client.send_goal(pub_action_msg)
        self.actionlib_client.wait_for_result()
        result = self.actionlib_client.get_result()
        return result
    
    def instuct_main(self, input_sentence):
        self.degree = -1
        input_sentence = self.convert_sentence(input_sentence)
        result = self.get_parsing_result(input_sentence)
        self.syntaxes = result.syntaxes
        self.lemma_lst = []
        for syntax in self.syntaxes:
            self.lemma_lst.append(syntax.lemma)

        if self.called_count == 0:
            response = self.first_call(input_sentence)
        else:
            response = self.interactive_call(input_sentence)
        if self.degree == -1:
            self.get_degree(input_sentence)
        
        self.called_count += 1
        return self.move_left, self.move_right, self.move_up, self.move_down, self.degree, response, self.called_count

    def first_call(self, input_sentence):
        # 上下左右
        response = self.get_clear_direction(input_sentence)
        if response:
            return response
        # お願い語
        request_flag = False
        for i, request_word in enumerate(self.request_words):
            if request_word in input_sentence:
                request_flag = True
                if request_word == "方が":
                    request_word = "方"
                if self.is_head_index_instruct(request_word):
                    return "上下左右どちらに動かしたら良いですか？"
        # トリガーワード
        if request_flag:
            for trigger_word in self.trigger_words:
                if trigger_word in input_sentence:
                    return "上下左右どちらに動かしたら良いですか？"
        self.init_param()
        return "終了"
    
    def interactive_call(self, input_sentence):
        # 上下左右
        response = self.get_clear_direction(input_sentence)
        if response:
            return response
        # 曖昧方向指示
        for indicator_direction in self.indicator_direction:
            if indicator_direction in input_sentence:
                self.called_count = -1
                return "代わりに置いてもらえますか？"
        # 否定
        if self.is_neg(input_sentence):
            if self.move_left == 1:
                self.move_right = 1
                self.move_left = 0
            elif self.move_right == 1:
                self.move_left = 1
                self.move_right = 0
            if self.move_up == 1:
                self.move_down = 1
                self.move_up = 0
            elif self.move_down == 1:
                self.move_up = 1
                self.move_down = 0
            if True in [self.move_left, self.move_right, self.move_up, self.move_down]:
                self.degree = 0
                return "風船を動かします"
        # 程度
        self.get_degree(input_sentence)
        if self.degree > -1:
            if True in [self.move_left, self.move_right, self.move_up, self.move_down]:
                return "風船を動かします"
        # 肯定
        for agree_word in self.agree_words:
            self.init_param()
            return "終了"
        return "上下左右どちらに動かしたら良いですか？"
    
    def get_clear_direction(self, input_sentence):
        keep_info = [self.move_left, self.move_right, self.move_up, self.move_down]
        self.move_left, self.move_right, self.move_up, self.move_down = False, False, False, False

        def get_prev_info(keep_info):
            self.move_left = keep_info[0]
            self.move_right = keep_info[1]
            self.move_up = keep_info[2]
            self.move_down = keep_info[3]
        
        def store_move_info(move_direction):
            if "左" in move_direction:
                self.move_left = True
            if "右" in move_direction:
                self.move_right = True
            if "上" in move_direction:
                self.move_up = True
            if "下" in move_direction:
                self.move_down = True
        
        not_neg_flag = False
        move_direction = ""
        for direction in self.clear_direction:
            if direction in input_sentence:
                move_direction += direction
                if not self.is_head_index_neg(direction):
                    not_neg_flag = True
                    store_move_info(direction)
        if not move_direction:
            get_prev_info(keep_info)
            return ""
        if not_neg_flag:
            return "風船を動かします"
        if not self.is_head_index_neg(move_direction):
            store_move_info(move_direction)
            return "風船を動かします"
        get_prev_info(keep_info)
        return ""

    def is_head_index_instruct(self, word):
        if not word in self.lemma_lst:
            return False
        word_pos = self.lemma_lst.index(word)
        depend_word_pos = self.syntaxes[word_pos].dependency_edge
        if self.syntaxes[depend_word_pos].name in self.action_verb:
            return True
        return False

    def is_head_index_neg(self, word):
        if not word in self.lemma_lst:
            return True
        word_pos = self.lemma_lst.index(word)
        for syntax in self.syntaxes:
            if syntax.dependency_edge == word_pos:
                if syntax.parse_label == 25:
                    return True
        return False

    def is_neg(self, input_sentence):
        for syntax in self.syntaxes:
            if syntax.parse_label == 25:
                return True
        for neg in self.excess_and_denial:
            if neg in input_sentence:
                return True
        return False

    def get_degree(self, input_sentence):
        self.degree = -1
        for degree in self.degree_words:
            if degree in input_sentence:
                if degree in ["もっと", "たくさん"]:
                    self.degree = 1
                elif degree in ["少し", "ちょっと"]:
                    self.degree = 0

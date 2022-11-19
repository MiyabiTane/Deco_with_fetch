#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import sys
import json
import requests

from instruct_human_to_robot import InstructChat

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
from std_msgs.msg import String, Bool
from deco_with_fetch.msg import InstructInfo

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')


class ChatNode(object):
    def __init__(self):
        self.apikey_path = rospy.get_param("~apikey_path", "")

        self.volume = 1.0
        self.command = 1
        self.sound = -3
        self.arg2 = 'ja'

        self.headers = {'content-type': 'application/json'}
        self.url = "https://api-mebo.dev/api"
        self.apikey = ""
        self.agentid = ""
        self.uid = ""
        self.set_mebo_param()

        self.instruct_flag = False
        self.instruct_chat = InstructChat()

        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.chat_cb)
        rospy.Subscriber("/instruct_flag", Bool, self.flag_cb)
        self.pub = rospy.Publisher("/text", String, queue_size=1)
        self.pub_instruct = rospy.Publisher("/instruct_info", InstructInfo, queue_size=1)
        self.actionlib_client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.actionlib_client.wait_for_server()

    def set_mebo_param(self):
        try:
            with open(self.apikey_path) as j:
                apikey_json = json.loads(j.read())
            self.apikey = apikey_json['apikey_mebo']
            self.agentid = apikey_json['agentid_mebo']
            self.uid = apikey_json['uid_mebo']
        except Exception as e:
            rospy.logerr('Could not find {}'.format(self.apikey_path))
            sys.exit(e)
    
    def make_chat_response(self, listern_text):
        rospy.loginfo("mebo input: {}".format(listern_text))
        input_data = json.dumps({'api_key': self.apikey, 'agent_id': self.agentid,
                                    'utterance': listern_text, 'uid': self.uid})
        response = requests.post(self.url, headers=self.headers, data=input_data, timeout=(3.0, 7.5))
        response_json = response.json()
        if 'bestResponse' not in response_json:
            best_response = "ごめんなさい、聞き取れませんでした"
        else:
            best_response = response_json['bestResponse']['utterance']
        rospy.loginfo("mebo output: {}".format(best_response))
        if "応答の取得に失敗しました" in best_response:
            best_response = "ごめんなさい、聞き取れませんでした"
        return best_response

    def chat_cb(self, msg):
        # Listern
        listen_text = ""
        if msg.transcript:
            for char in msg.transcript:
                listen_text += char

        instruct_msg = InstructInfo()
        best_response = "雑談"
        if self.instruct_flag:
            move_left, move_right, move_up, move_down, degree, best_response, count = self.instruct_chat.instuct_main(listen_text)
            if best_response == "終了":
                best_response = "雑談"
            else:
                # Publish info of which to move
                instruct_msg.called_count = count
                instruct_msg.message = best_response
                instruct_msg.move_up = move_up
                instruct_msg.move_down = move_down
                instruct_msg.move_left = move_left
                instruct_msg.move_right = move_right
                instruct_msg.move_degree = degree
                if best_response == "代わりに置いてもらえますか？":
                    instruct_msg.is_finish = True
                else:
                    instruct_msg.is_finish = False
                self.pub_instruct.publish(instruct_msg)

        if best_response == "雑談":
            # Instruct
            instruct_msg.is_finish = True
            self.pub_instruct.publish(instruct_msg)

            best_response = self.make_chat_response(listen_text)

            # Publish for eyebrows expression
            pub_msg = String()
            pub_msg.data = best_response
            self.pub.publish(pub_msg)
            # Speak
            rospy.sleep(0.5)
            speak_msg = SoundRequestGoal()
            speak_msg.sound_request.volume = self.volume
            speak_msg.sound_request.command = self.command
            speak_msg.sound_request.sound = self.sound
            speak_msg.sound_request.arg = best_response
            speak_msg.sound_request.arg2 = self.arg2
            self.actionlib_client.send_goal(speak_msg)
            # self.actionlib_client.wait_for_result()


    def flag_cb(self, msg):
        self.instruct_flag = msg.data

if __name__ == '__main__':
    rospy.init_node("chat_node")
    chat_node = ChatNode()
    rospy.spin()

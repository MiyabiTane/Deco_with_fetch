#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from dialogflow_task_executive.msg import DialogResponse
from std_msgs.msg import Int32

CONFIDENCE_TH = 0.6

class EyebrowsTopicRelay(object):
    def __init__(self):
        self.listen_text = ""

        rospy.Subscriber("/dialog_response", DialogResponse, self.diag_cb)
        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_cb)
        self.pub = rospy.Publisher("/eyebrows_expression/input", Int32, queue_size=1)

    def speech_cb(self, msg):
        # dialogflow responses not only /text but also /speech_to_text
        self.listen_text = ""
        if msg.transcript:
            for char in msg.transcript:
                self.listen_text += char

    def diag_cb(self, msg):
        pub_num = 0
        action_num_dict = {"Happy": 1, "Relieved": 2, "Smirking": 3, "Astonished": 4,
                            "Unpleasant": 5, "Angry": 6, "Flushed": 7, "Fearful": 8,
                            "Love": 9, "Squinting": 10, "Boring": 11, "Cry": 12}

        if msg.query != self.listen_text:
            if msg.action in action_num_dict:
                if msg.intent_score > CONFIDENCE_TH:
                    pub_num = action_num_dict[msg.action]
        if pub_num != 0:
            pub_msg = Int32()
            pub_msg.data = pub_num
            self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("eyebrows_topic")
    eyebrows_topic_relay = EyebrowsTopicRelay()
    rospy.spin()

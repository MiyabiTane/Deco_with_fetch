#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String, Bool

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

class IsSuggest(object):
    def __init__(self):
        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_cb)
        self.pub = rospy.Publisher("/is_suggest", Bool, queue_size=1)

    def speech_cb(self, msg):
        listen_text = ""
        if msg.transcript:
            for char in msg.transcript:
                listen_text += char

        pub_msg = Bool()
        pub_msg.data = False
        if "どこ" in listen_text:
            if "置く" in listen_text or "置い" in listen_text:
                pub_msg.data = True
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node("is_suggest")
    is_suggest = IsSuggest()
    rospy.spin()

#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_msgs.msg import String

class NaoqiAnimatedSpeech(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_animated_speech')
        self.connectNaoQi()
        self.rate = rospy.Rate(10) 
        rospy.Subscriber("animated_speech", String, self.sayCallback, queue_size=10)
        rospy.loginfo("naoqi_animated_speech initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.animatedSpeechProxy = self.get_proxy("ALAnimatedSpeech")
        if self.animatedSpeechProxy is None:
            rospy.logerr("Could not get a proxy to ALAnimatedSpeech")
            exit(1)

    def sayCallback(self, data):
        self.animatedSpeechProxy.say(data.data)
                
if __name__ == '__main__':
    animated_speech = NaoqiAnimatedSpeech()
    rospy.spin()

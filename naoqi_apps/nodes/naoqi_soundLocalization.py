#!/usr/bin/env python
#                                                                             
#  Copyright 2016 Aldebaran                                                   
#                                                                             
#  Licensed under the Apache License, Version 2.0 (the "License");            
#  you may not use this file except in compliance with the License.           
#  You may obtain a copy of the License at                                    
#                                                                             
#      http://www.apache.org/licenses/LICENSE-2.0                             
#                                                                             
#  Unless required by applicable law or agreed to in writing, software        
#  distributed under the License is distributed on an "AS IS" BASIS,          
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   
#  See the License for the specific language governing permissions and        
#  limitations under the License.                                             
#                                                                             
# 
import rospy
from naoqi_bridge_msgs.msg import SoundLocated
from naoqi_driver.naoqi_node import NaoqiNode

class NaoqiSoundLocalization (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_soundLocalization')
        self.connectNaoQi()
        self.soundLocalization = SoundLocated()
        self.soundLocalizationPub = rospy.Publisher("soundLocated", SoundLocated, queue_size=10)
        rospy.loginfo("naoqi_soundLocalization is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        
        if self.memProxy is None:
            exit(1)
       
    def run(self):
        while self.is_looping():
            try:
                r = rospy.Rate(5)
                sound = self.memProxy.getData("ALSoundLocalization/SoundLocated")
                if (len(sound)) > 0:
                    self.soundLocalization.header.stamp = rospy.get_rostime()
                    self.soundLocalization.threshold.data = sound[1][0]
                    for i in range(0, 2):
                        self.soundLocalization.location.data.append(sound[1][i+1])
                    for i in range(0, 6):
                        self.soundLocalization.head_position.data.append(sound[2][i])
                    self.soundLocalizationPub.publish(self.soundLocalization)
                    
                    for i in range(len(self.soundLocalization.location.data)):
                        del self.soundLocalization.location.data[0]
                    
                    for i in range(len(self.soundLocalization.head_position.data)):
                        del self.soundLocalization.head_position.data[0]
                        
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    soundLocalization = NaoqiSoundLocalization()
    soundLocalization.start()
    rospy.spin()

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
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import ( 
     SetArmsEnabledResponse,
     SetArmsEnabled,)

class NaoqiMotion(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_motion')
        self.connectNaoQi()
        
        self.setArmsEnabledSrv = rospy.Service("set_move_arms_enabled", SetArmsEnabled, self.handleSetMoveArmsEnabledSrv)
        ##self.getArmsEnabledSrv = rospy.Service("get_move_arms_enabled", SetArmsEnabled, self.handleGetMoveArmsEnabledSrv)
        rospy.loginfo("naoqi_motion initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.motionProxy = self.get_proxy("ALMotion")
        if self.memProxy is None or self.motionProxy is None:
            exit(1)
        
    def handleSetMoveArmsEnabledSrv(self, req):
        try:
            self.motionProxy.setMoveArmsEnabled(req.left_arm, req.right_arm)
            rospy.loginfo("Left Arm is set to " + str(req.left_arm) + "and Right Arm is set to " + str(req.right_arm))
            return SetArmsEnabledResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    motion = NaoqiMotion()
    motion.start()
    rospy.spin()

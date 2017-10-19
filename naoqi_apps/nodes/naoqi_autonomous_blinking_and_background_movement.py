#!/usr/bin/env python

#                                                                             
#  Copyright 2017 Aldebaran                                                   
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
from std_srvs.srv import (
    SetBoolResponse,
    SetBool,
    TriggerResponse,
    Trigger
)

class NaoqiAutonomousBlinking_and_BackgroundMovement(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_autonomous_blinking_and_background_movement')
        self.connectNaoQi()
        
        self.getAutonomousBlinkingEnabledSrv = rospy.Service("autonomous_blinking_get_enabled", Trigger, self.handleGetAutonomousBlinkingEnabled)
        self.setAutonomousBlinkingEnabledSrv = rospy.Service("autonomous_blinking_set_enabled", SetBool, self.handleSetAutonomousBlinkingEnabled)
        self.getBackgroundMovementEnabledSrv = rospy.Service("background_movement_get_enabled", Trigger, self.handleGetBackgroundMovementEnabled)
        self.setBackgroundMovementEnabledSrv = rospy.Service("background_movement_set_enabled", SetBool, self.handleSetBackgroundMovementEnabled)
        rospy.loginfo("naoqi_autonomous_blinking_and_background_movement initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.abProxy = self.get_proxy("ALAutonomousBlinking")
        self.bmProxy = self.get_proxy("ALBackgroundMovement")
        if self.abProxy is None or self.bmProxy is None:
            exit(1)
        
    def handleGetAutonomousBlinkingEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.abProxy.isEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetBackgroundMovementEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.bmProxy.isEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetAutonomousBlinkingEnabled(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.abProxy.setEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetBackgroundMovementEnabled(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.bmProxy.setEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    autonomousBlinking_and_backgroundMovement = NaoqiAutonomousBlinking_and_BackgroundMovement()
    autonomousBlinking_and_backgroundMovement.start()
    rospy.spin()

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
    CreateGroupResponse,
    CreateGroup,
    #EarLedsSetAngleResponse,
    #EarLedsSetAngle,
    #FadeResponse,
    #Fade,
    #FadeListRGBResponse,
    #FadeListRGB,
    #FadeRGBWithColorCodeResponse,
    #FadeRGBWithColorCode,
    FadeRGBWithNameResponse,
    FadeRGBWithName,
    #FadeRGBWithHexadecimalColorCodeResponse,
    #FadeRGBWithHexadecimalColorCode,
    #GetIntensityResponse,
    #GetIntensity,
    #ListGroupResponse,
    #ListGroup,
    #OffResponse,
    #Off,
    #OnResponse,
    #On
    #RandomEyesResponse,
    #RandomEyes,
    #RastaResponse,
    #Rasta,
    ResetResponse,
    Reset,
    #SetIntensityResponse,
    #SetIntensity
)
# ListGroups : Empty
# ListLeds : Empty
# input: string type output: none (or success) srv is many, I can reduce srv files.

class NaoqiLeds(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_leds')
        self.connectNaoQi()
        
        self.CreateGroupSrv = rospy.Service("create_group", CreateGroup, self.handleCreateGroupSrv)
        self.FadeRGBWithNameSrv = rospy.Service("fade_rgb_with_name", FadeRGBWithName, self.handleFadeRGBWithNameSrv)
        self.ResetSrv = rospy.Service("reset", Reset, self.handleResetSrv)
        rospy.loginfo("naoqi_leds initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.ledsProxy = self.get_proxy("ALLeds")
        if self.ledsProxy is None:
            exit(1)
    
    def handleCreateGroupSrv(self, req):
        res = CreateGroupResponse()
        res.success = False
        try:
            self.ledsProxy.createGroup(req.group_name, req.led_names)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleFadeRGBWithNameSrv(self, req):
        res = FadeRGBWithNameResponse()
        res.success = False
        try:
            self.ledsProxy.fadeRGB(req.led_name, req.color_name, req.time_to_fade)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleResetSrv(self, req):
        res = ResetResponse()
        res.success = False
        try:
            self.ledsProxy.reset(req.led_name)
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
                print "Error accessing ALLeds, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    leds = NaoqiLeds()
    leds.start()
    rospy.spin()

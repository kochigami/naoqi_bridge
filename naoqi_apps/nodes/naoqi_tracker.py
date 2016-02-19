#!/usr/bin/env python

#                                                                             
#  Copyright 2015 Aldebaran                                                   
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
from naoqi import (ALBroker, ALProxy, ALModule)
from std_srvs.srv import( Empty, EmptyResponse )

class NaoqiTracker(NaoqiNode):
    def __init__( self ):
        NaoqiNode.__init__( self, 'naoqi_tracker' )
        self.connectNaoQi()

        self.faceTracker_srv = rospy.Service("face_tracker", Empty, self.faceTracker )
        self.stopTracker_srv = rospy.Service("stop_tracker", Empty, self.stopTracker )
        self.trackerFlag = False

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.basicAwarenessProxy = self.get_proxy("ALBasicAwareness")
        if self.basicAwarenessProxy is None:
            exit(1)

    def faceTracker( self, request = None ):
        try:
            self.trackerFlag = True
            self.basicAwarenessProxy.setEngagementMode("Unengaged")
            self.basicAwarenessProxy.setTrackingMode("Head")
            self.basicAwarenessProxy.setStimulusDetectionEnabled("Sound", True)
            self.basicAwarenessProxy.setStimulusDetectionEnabled("Movement", True)
            self.basicAwarenessProxy.setStimulusDetectionEnabled("People", True)
            self.basicAwarenessProxy.setStimulusDetectionEnabled("Touch", True)
            self.basicAwarenessProxy.setEnabled(True)
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def stopTracker( self, request = None ):
        try:
            self.basicAwarenessProxy.setEnabled(False)
            self.trackerFlag = False
            return EmptyResponse()
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
    Tracker = NaoqiTracker()
    Tracker.start()
    rospy.spin()

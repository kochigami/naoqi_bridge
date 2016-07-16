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
#from std_msgs.msg import Float64
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty)

class NaoqiTracker (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tracker')
        self.connectNaoQi()
        self.moduleName = "Tracker"
        self.subscribeDone = False
        self.People_ID = None
        self.targetName = "People" # dynamic reconfigure
        self.mode = "Head" # dynamic reconfigure 
        self.effector = "None" # dynamic reconfigure
        self.startTrackerSrv = rospy.Service("start_tracker", Empty, self.handleStartTrackerSrv)
        self.stopTrackerSrv = rospy.Service("stop_tracker", Empty, self.handleStopTrackerSrv)
        rospy.loginfo("naoqi_tracker is initialized")
        
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.trackerProxy = self.get_proxy("ALTracker")
        
        if self.memProxy is None:
            exit(1)
        if self.trackerProxy is None:
            exit(1)

    def handleStartTrackerSrv (self, req):
        try:
            self.trackerProxy.registerTarget(self.targetName, self.People_ID)
            #self.trackerProxy.setRelativePosition([-self.distanceX, self.distanceY, self.angleWz, self.thresholdX, self.thresholdY, self.thresholdWz])
            self.trackerProxy.setEffector(self.effector)
            self.trackerProxy.setRelativePosition([-0.3, 0.0, 0.0, 0.1, 0.1, 0.3]) # dynamic reconfigure
            self.trackerProxy.setMode(self.mode)
            self.trackerProxy.track(self.targetName) #Start tracker
            
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStopTrackerSrv (self, req):
        try:
            if self.subscribeDone:
                self.memProxy.unsubscribeToEvent("ALTracker/TargetLost", self.moduleName)
                self.memProxy.unsubscribeToEvent("ALTracker/TargetReached", self.moduleName)
                self.subscribeDone = False
            self.trackerProxy.setEffector("None")
            self.trackerProxy.stopTracker()
            self.trackerProxy.unregisterTarget(self.targetName)
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                self.memProxy.subscribeToEvent("ALTracker/TargetLost", self.moduleName, "onTargetLost")
                self.memProxy.subscribeToEvent("ALTracker/TargetReached", self.moduleName, "onTargetReached")
                self.subscribeDone = True
                
                data_list = self.memProxy.getDataList("PeoplePerception")
                for i in range (len(data_list)):
                    if data_list[i] == "PeoplePerception/PeopleList":
                        People_ID_list = self.memProxy.getData("PeoplePerception/PeopleList")
                        if (len(People_ID_list)) > 0:
                            self.People_ID = People_ID_list[0]
                            self.trackerProxy.registerTarget(self.targetName, self.People_ID)
                            
                self.memProxy.subscribeToEvent("ALTracker/ActiveTargetChanged", self.moduleName, "onTargetChanged")
                        
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

    def onTargetChanged(self, key, value, message):
        if value == self.targetName and not self.subscribeDone:
            self.memoProxy.subscribeToEvent("ALTracker/TargetLost", self.moduleName, "onTargetLost")
            self.memProxy.subscribeToEvent("ALTracker/TargetReached", self.moduleName, "onTargetReached")
            self.subscribeDone = True
        elif value != self.targetName and self.subscribeDone:
            self.memProxy.unsubscribeToEvent("ALTracker/TargetLost", self.moduleName)
            self.memProxy.unsubscribeToEvent("ALTracker/TargetReached", self.moduleName)
            self.subscribeDone = False

    def onTargetLost(self, key, value, message):
        print "target is lost"
        #self.targetLost()

    def onTargetReached(self, key, value, message):
        #self.targetReached()
        print "target is reached"

if __name__ == '__main__':
    tracker = NaoqiTracker()
    tracker.start()
    rospy.spin()

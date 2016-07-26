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
from dynamic_reconfigure.server import Server as ReConfServer
import dynamic_reconfigure.client
from naoqi_apps.cfg import NaoqiTrackerConfig as NodeConfig # todo
from std_msgs.msg import Bool
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
        self.targetName = "Face" #"People" 
        self.mode = "Head" 
        self.effector = "None"      
        self.conf = None
        self.tracker = False
        # Start reconfigure server  
        self.reconf_server = ReConfServer(NodeConfig, self.reconfigure)
        # Client for receiving the new information                             
        self.reconf_client = dynamic_reconfigure.client.Client("naoqi_tracker")

        
        self.startTrackerSrv = rospy.Service("start_tracker", Empty, self.handleStartTrackerSrv)
        self.stopTrackerSrv = rospy.Service("stop_tracker", Empty, self.handleStopTrackerSrv)
        self.targetLostPub = rospy.Publisher("target_lost", Bool, queue_size=10)
        self.targetReachedPub = rospy.Publisher("target_reached", Bool, queue_size=10)
        self.targetLost = Bool()
        self.targetReached = Bool()
        
        rospy.loginfo("naoqi_tracker is initialized")
        
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.trackerProxy = self.get_proxy("ALTracker")
        
        if self.memProxy is None:
            exit(1)
        if self.trackerProxy is None:
            exit(1)

    def handleStartTrackerSrv (self, req = None):
        try:
            self.targetName = self.conf["targetName"]
            if self.targetName == "People":
                self.trackerProxy.registerTarget(self.targetName, self.People_ID)
            if self.targetName == "Face":
                self.trackerProxy.registerTarget(self.targetName, self.conf["width"])
            self.trackerProxy.setEffector(self.conf["effector"])
            self.trackerProxy.setRelativePosition([(- self.conf["distanceX"]), self.conf["distanceY"], self.conf["distanceWz"], self.conf["thresholdX"], self.conf["thresholdY"], self.conf["thresholdWz"]]) 

            self.trackerProxy.setMode(self.conf["mode"])
            self.trackerProxy.track(self.targetName) #Start tracker
            self.tracker = True
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStopTrackerSrv (self, req = None):
        try:
            if self.subscribeDone:
                self.memProxy.unsubscribeToEvent("ALTracker/TargetLost", self.moduleName)
                self.memProxy.unsubscribeToEvent("ALTracker/TargetReached", self.moduleName)
                self.subscribeDone = False
            self.trackerProxy.setEffector("None")
            self.trackerProxy.stopTracker()
            self.trackerProxy.unregisterTarget(self.targetName)
            self.tracker = False
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def reconfigure(self, request, level):
        rospy.loginfo("""Reconfigure Request: {mode}, {effector}, {width}, {distanceX}, {thresholdX}, {distanceY}, {thresholdY}, {distanceWz}, {thresholdWz}""".format(**request))
        #return request
        newConf = {}
        #Copy values
        newConf["mode"] = request["mode"]
        newConf["targetName"] = request["targetName"]
        newConf["effector"] = request["effector"]
        newConf["width"] = request["width"]
        newConf["distanceX"] = request["distanceX"]
        newConf["thresholdX"] = request["thresholdX"]
        newConf["distanceY"] = request["distanceY"]
        newConf["thresholdY"] = request["thresholdY"]
        newConf["distanceWz"] = request["distanceWz"]
        newConf["thresholdWz"] = request["thresholdWz"]

        # Check and update values                                              
        if not newConf["mode"]:
            newConf["mode"] = self.trackerProxy.getMode()
        elif newConf["mode"] not in self.trackerProxy.getAvailableModes():
            rospy.logwarn(
                "Unknown mode '{}'. Using current mode instead".format(
                    newConf["mode"] ) )
            rospy.loginfo("Modes available: {}".format(
                self.trackerProxy.getAvailablModes()))
            newConf["mode"] = self.trackerProxy.getMode()
        if not newConf["targetName"]:
            newConf["targetName"] = self.trackerProxy.getActiveTarget()
        elif newConf["targetName"] not in ["RedBall", "Face", "LandMark", "LandMarks", "People", "Sound"]:
            rospy.logwarn(
                "Unknown target name '{}'. Using current targte name instead".format(
                    newConf["targetName"] ) )
            rospy.loginfo("Target Names available: RedBall, Face, LandMark, LandMarks, People, Sound")
            newConf["mode"] = self.trackerProxy.getMode()
        if not newConf["effector"]:
            newConf["effector"] = self.trackerProxy.getEffector()
        elif newConf["effector"] not in ["None", "Arms", "LArm", "RArm"]:
            newConf["effector"] = self.trackerProxy.getEffector()
            rospy.logwarn(
                "Unknown effector '{}'. Using current effector instead".format(
                    newConf["effector"] ) )
            rospy.loginfo("Effectors available: None, Arms, LArm, RArm")

        # # If first time and parameter not explicitly set                       
        # if not self.conf and not rospy.has_param("~volume"):
        #     newConf["volume"] = self.audio.getOutputVolume()

        # # if srw is running and the vocabulary request is invalid, ignore it   
        # if self.srw and not Util.parse_vocabulary(newConf["vocabulary"]):
        #     rospy.logwarn("Empty vocabulary. Using current vocabulary instead")
        #     newConf["vocabulary"] = self.conf["vocabulary"]

        #  Check if we need to restart tracker
        if self.tracker and self.conf and (
                newConf["mode"] != self.conf["mode"] or
                newConf["targetName"] != self.conf["targetName"] or
                newConf["effector"] != self.conf["effector"] or
                newConf["width"] != self.conf["width"] or
                newConf["distanceX"] != self.conf["distanceX"] or
                newConf["distanceY"] != self.conf["distanceY"] or
                newConf["distanceWz"] != self.conf["distanceWz"] or
                newConf["thresholdX"] != self.conf["thresholdX"] or
                newConf["thresholdY"] != self.conf["thresholdY"] or
                newConf["thresholdWz"] != self.conf["thresholdWz"]):
            need_to_restart_tracker = True
        else:
            need_to_restart_tracker = False
            self.conf = newConf

        #If we have enabled the tracker wrapper, reconfigure it     
        if need_to_restart_tracker:
            self.conf = newConf
            self.handleStopTrackerSrv()
            self.handleStartTrackerSrv()
        return self.conf

    def run(self):
        while self.is_looping():
            try:
                self.memProxy.subscribeToEvent("ALTracker/TargetLost", self.moduleName, "onTargetLost")
                self.memProxy.subscribeToEvent("ALTracker/TargetReached", self.moduleName, "onTargetReached")
                self.subscribeDone = True
                
                if self.targetName == "People":
                    data_list = self.memProxy.getDataList("PeoplePerception")
                    for i in range (len(data_list)):
                        if data_list[i] == "PeoplePerception/VisiblePeopleList":
                            People_ID_list = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
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
            self.memProxy.subscribeToEvent("ALTracker/TargetLost", self.moduleName, "onTargetLost")
            self.memProxy.subscribeToEvent("ALTracker/TargetReached", self.moduleName, "onTargetReached")
            self.subscribeDone = True
        elif value != self.targetName and self.subscribeDone:
            self.memProxy.unsubscribeToEvent("ALTracker/TargetLost", self.moduleName)
            self.memProxy.unsubscribeToEvent("ALTracker/TargetReached", self.moduleName)
            self.subscribeDone = False
    
    # need to modify
    def onTargetLost(self, key, value, message):
        self.targetLost.data = True
        self.targetLostPub.publish(self.targetLost)
        print "target is lost"
    # need to modify
    def onTargetReached(self, key, value, message):
        self.targetReached.data = True
        self.targetReachedPub.publish(self.targetReached)
        print "target is reached"

if __name__ == '__main__':
    tracker = NaoqiTracker()
    tracker.start()
    rospy.spin()

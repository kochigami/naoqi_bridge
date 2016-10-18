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
from naoqi_bridge_msgs.msg import PeopleDetected
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.srv import (
    GetMaximumDetectionRangeResponse,
    GetMaximumDetectionRange,
    GetTimeBeforePersonDisappearsResponse,
    GetTimeBeforePersonDisappears,
    GetTimeBeforeVisiblePersonDisappearsResponse,
    GetTimeBeforeVisiblePersonDisappears,
    IsFaceDetectionEnabledResponse,
    IsFaceDetectionEnabled,
    IsFastModeEnabledResponse,
    IsFastModeEnabled,
    IsGraphicalDisplayEnabledResponse,
    IsGraphicalDisplayEnabled,
    IsMovementDetectionEnabledResponse,
    IsMovementDetectionEnabled,
    SetFastModeEnabledResponse,
    SetFastModeEnabled,
    SetGraphicalDisplayEnabledResponse,
    SetGraphicalDisplayEnabled,
    SetMaximumDetectionRangeResponse,
    SetMaximumDetectionRange,
    SetMovementDetectionEnabledResponse,
    SetMovementDetectionEnabled,
    SetTimeBeforePersonDisappearsResponse,
    SetTimeBeforePersonDisappears,
    SetTimeBeforeVisiblePersonDisappearsResponse,
    SetTimeBeforeVisiblePersonDisappears
    )

class NaoqiPeoplePerception (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_peoplePerception')
        self.connectNaoQi()
        self.peopleDetected = PeopleDetected()
        #self.prePeopleDetected = None
        self.peopleDetectedPub = rospy.Publisher("peopleDetected", PeopleDetected, queue_size=10)
        self.getMaximumDetectionRangeSrv = rospy.Service("get_maximum_detection_range", GetMaximumDetectionRange, self.handleGetMaximumDetectionRange)
        self.getTimeBeforePersonDisappearsSrv = rospy.Service("get_time_before_person_disappears", GetTimeBeforePersonDisappears , self.handleGetTimeBeforePersonDisappears)
        self.getTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("get_time_before_visible_person_disappears", GetTimeBeforeVisiblePersonDisappears , self.handleGetTimeBeforeVisiblePersonDisappears)
        self.isFaceDetectionEnabledSrv = rospy.Service("is_face_detection_enabled", IsFaceDetectionEnabled, self.handleIsFaceDetectionEnabled)
        self.isFastModeEnabledSrv = rospy.Service("get_fast_mode_enabled", IsFastModeEnabled, self.handleIsFastModeEnabled)
        self.isGraphicalDisplayEnabledSrv = rospy.Service("is_graphical_display_enabled", IsGraphicalDisplayEnabled, self.handleIsGraphicalDisplayEnabled)
        self.isMovementDetectionEnabledSrv = rospy.Service("get_movement_detection_enabled", IsMovementDetectionEnabled, self.handleIsMovementDetectionEnabled)
        self.resetPopulationSrv = rospy.Service("reset_population", Empty, self.handleResetPopulation)
        self.setFastModeEnabledSrv = rospy.Service("set_fast_mode_enabled", SetFastModeEnabled, self.handleSetFastModeEnabled)
        self.setGraphicalDisplayEnabledSrv = rospy.Service("set_graphical_display_enabled", SetGraphicalDisplayEnabled, self.handleSetGraphicalDisplayEnabled)
        self.setMaximumDetectionRangeSrv = rospy.Service("set_maximum_detection_range_enabled", SetMaximumDetectionRange, self.handleSetMaximumDetectionRange)
        self.setMovementDetectionEnabledSrv = rospy.Service("set_move_detection_enabled", SetMovementDetectionEnabled, self.handleSetMovementDetectionEnabled)
        self.setTimeBeforePersonDisappearsSrv = rospy.Service("set_time_before_person_disappears", SetTimeBeforePersonDisappears, self.handleSetTimeBeforePersonDisappears)
        self.setTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("set_time_before_visible_person_disappears", SetTimeBeforeVisiblePersonDisappears, self.handleSetTimeBeforeVisiblePersonDisappears)
        rospy.loginfo("naoqi_peoplePerception is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.peoplePerceptionProxy = self.get_proxy("ALPeoplePerception")
        if self.memProxy is None or self.peoplePerceptionProxy is None:
            exit(1)
            
    def handleGetMaximumDetectionRange(self, req):
        try:
            res = GetMaximumDetectionRangeResponse()
            res.ranges = self.peoplePerceptionProxy.getMaximumDetectionRange()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforePersonDisappears(self, req):
        try:
            res = GetTimeBeforePersonDisappearsResponse()
            res.time_before_person_disappears = self.peoplePerceptionProxy.getTimeBeforePersonDisappears()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforeVisiblePersonDisappears(self, req):
        try:
            res = GetTimeBeforeVisiblePersonDisappearsResponse()
            res.time_before_visible_person_disappears = self.peoplePerceptionProxy.getTimeBeforeVisiblePersonDisappears()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleIsFaceDetectionEnabled(self, req):
        try:
            res = IsFaceDetectionEnabledResponse()
            res.status = self.peoplePerceptionProxy.isFaceDetectionEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsFastModeEnabled(self, req):
        try:
            res = IsFastModeEnabledResponse()
            res.status = self.peoplePerceptionProxy.isFastModeEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsGraphicalDisplayEnabled(self, req):
        try:
            res = IsGraphicalDisplayEnabledResponse()
            res.status = self.peoplePerceptionProxy.isGraphicalDisplayEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsMovementDetectionEnabled(self, req):
        try:
            res = IsMovementDetectionEnabledResponse()
            res.status = self.peoplePerceptionProxy.isMovementDetectionEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleResetPopulation(self, req):
        try:
            res = EmptyResponse()
            self.peoplePerceptionProxy.resetPopulation()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetFastModeEnabled(self, req):
        res = SetFastModeEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setFastModeEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetGraphicalDisplayEnabled(self, req):
        res = SetGraphicalDisplayEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setGraphicalDisplayEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumDetectionRange(self, req):
        res = SetMaximumDetectionRangeResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMaximumDetectionRange(req.ranges)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMovementDetectionEnabled(self, req):
        res = SetMovementDetectionEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMovementDetectionEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforePersonDisappears(self, req):
        res = SetTimeBeforePersonDisappearsResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforePersonDisappears(req.seconds)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforeVisiblePersonDisappears(self, req):
        res = SetTimeBeforeVisiblePersonDisappearsResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforeVisiblePersonDisappears(req.seconds)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                r = rospy.Rate(5)
                data_list = self.memProxy.getDataList("VisiblePeopleList")
                for i in range (len(data_list)):
                    if data_list[i] == "PeoplePerception/VisiblePeopleList":
                        People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                        if (len(People_ID)) > 0:
                            distance_event_name = "PeoplePerception/Person/" + str(People_ID[0]) + "/Distance"
                            angles_yaw_pitch_event_name = "PeoplePerception/Person/" + str(People_ID[0]) + "/AnglesYawPitch"
                            data_list = self.memProxy.getDataList("Person")
                            for i in range (len(data_list)):
                                if data_list[i] == distance_event_name:
                                    distance = self.memProxy.getData(distance_event_name)
                                if data_list[i] == angles_yaw_pitch_event_name:
                                    angles_yaw_pitch = self.memProxy.getData(angles_yaw_pitch_event_name)

                            self.peopleDetected.header.stamp = rospy.get_rostime()
                            self.peopleDetected.people_id.data = People_ID[0]
                            if distance:
                                self.peopleDetected.distance.data = distance
                                distance = None
                        
                            if angles_yaw_pitch:
                                for i in range (len(angles_yaw_pitch)):
                                    self.peopleDetected.angles.data.append(angles_yaw_pitch[i])
                                angles_yaw_pitch = None
                               
                            self.peopleDetectedPub.publish(self.peopleDetected)

                            for i in range(len(self.peopleDetected.angles.data)):
                                del self.peopleDetected.angles.data[0]
                            self.peopleDetected.people_id.data = None
                            self.peopleDetected.distance.data =None

                # PeopleDetectedList = self.memProxy.getData("PeoplePerception/PeopleDetected")
                # print PeopleDetectedList
                # self.peopleDetected.header.stamp = rospy.get_rostime()
                # People_list = PeopleDetectedList[1]
                # for i in range(len(People_list)):
                #     One_People = People_list[i]
                #     for i in range(len(One_People)):
                #         self.peopleDetected.people_data.data.append(One_People[i])
                # for i in range(len(PeopleDetectedList[2])):
                #     self.peopleDetected.cameraPose_inTorsoFrame.data.append((PeopleDetectedList[2])[i])
                # for i in range(len(PeopleDetectedList[3])):
                #     self.peopleDetected.cameraPose_inRobotFrame.data.append((PeopleDetectedList[3])[i])
                # self.peopleDetected.camera_id.data = PeopleDetectedList[4]
                
                # # data_list = self.memProxy.getDataList("VisiblePeopleList")
                # # for i in range (len(data_list)):
                # #     if data_list[i] == "PeoplePerception/VisiblePeopleList":
                # #         People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                # #      #   if (len(People_ID)) > 0:
                            
                # if (self.prePeopleDetected):
                #     for i in range(len(PeopleDetectedList[1])):
                #         #print (PeopleDetectedList[1])[i]
                #         #print self.prePeopleDetected.people_data.data
                #         if ((PeopleDetectedList[1])[i] != self.prePeopleDetected.people_data.data):
                #             self.peopleDetectedPub.publish(self.peopleDetected)
                # else:
                #     self.peopleDetectedPub.publish(self.peopleDetected)
                # self.prePeopleDetected = self.peopleDetected
                # #print self.prePeopleDetected
                # #print self.peopleDetected
                # r.sleep()
                
                # for i in range(len(self.peopleDetected.people_data.data)):
                #     del self.peopleDetected.people_data.data[0]
                # self.peopleDetected.cameraPose_inTorsoFrame.data = []
                # self.peopleDetected.cameraPose_inRobotFrame.data = []
                # self.peopleDetected.camera_id.data = None
                                
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    peoplePerception.start()
    rospy.spin()

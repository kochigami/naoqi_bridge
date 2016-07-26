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
from std_msgs.msg import Float64MultiArray
from naoqi_bridge_msgs.msg import IsLookingAtRobot
from naoqi_driver.naoqi_node import NaoqiNode

class NaoqiGazeAnalysis (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_gazeAnalysis')
        self.connectNaoQi()
        self.gaze_direction = Float64MultiArray()
        self.head_angles = Float64MultiArray()
        self.looking_at_robot = IsLookingAtRobot()
        self.gazeDirectionPub = rospy.Publisher("gaze_direction", Float64MultiArray, queue_size=10)
        self.headAnglesPub = rospy.Publisher("head_angles", Float64MultiArray, queue_size=10)        
        self.lookingAtRobotPub = rospy.Publisher("is_looking_at_robot", IsLookingAtRobot, queue_size=10)
        rospy.loginfo("naoqi_gazeAnalysis is initialized")
        self.pre_gaze_direction_array = [0.0, 0.0]
        self.pre_head_angles_array = [0.0, 0.0, 0.0]
        self.publish_flag = False

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                data_list = self.memProxy.getDataList("PeoplePerception/VisiblePeopleList")
                if (len(data_list)) > 0:
                    for i in range (len(data_list)):
                        if data_list[i] == "PeoplePerception/VisiblePeopleList":
                            People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                            #if People_ID != -1:
                            if (len(People_ID)) > 0:
                                gaze_direction_event_name = "PeoplePerception/Person/" + str(People_ID[0]) + "/GazeDirection"
                                head_angles_event_name = "PeoplePerception/Person/" + str(People_ID[0]) + "/HeadAngles"
                                is_looking_at_robot_event_name = "PeoplePerception/Person/" + str(People_ID[0]) + "/IsLookingAtRobot"
                                data_list = self.memProxy.getDataList("Person")
                                for i in range (len(data_list)):
                                    if data_list[i] == gaze_direction_event_name:
                                        gaze_direction_array = self.memProxy.getData(gaze_direction_event_name)
                                        if len(gaze_direction_array) > 0:
                                            for i in range (len(gaze_direction_array)):
                                                self.gaze_direction.data.append(gaze_direction_array[i]) 
                                            for i in range (len(gaze_direction_array)):
                                                if gaze_direction_array[i] != self.pre_gaze_direction_array[i]:
                                                    self.publish_flag = True
                                            if self.publish_flag == True:
                                                self.gazeDirectionPub.publish(self.gaze_direction)
                                            for i in range (len(gaze_direction_array)):
                                                self.pre_gaze_direction_array[i] = gaze_direction_array[i] 
                                            self.gaze_direction_array = []
                                            self.gaze_direction.data = []
                                            self.publish_flag = False
                            
                                data_list = self.memProxy.getDataList("Person")
                                for i in range (len(data_list)):
                                    if data_list[i] == head_angles_event_name:
                                        head_angles_array = self.memProxy.getData(head_angles_event_name)
                                        if len(head_angles_array) > 0:
                                            for i in range (len(head_angles_array)):
                                                self.head_angles.data.append(head_angles_array[i]) 
                                            for i in range (len(head_angles_array)):
                                                if head_angles_array[i] != self.pre_head_angles_array[i]:
                                                    self.publish_flag = True
                                            if self.publish_flag == True:
                                                self.headAnglesPub.publish(self.head_angles)
                                            for i in range (len(head_angles_array)):
                                                self.pre_head_angles_array[i] = head_angles_array[i] 
                                            self.head_angles_array = []
                                            self.head_angles.data = []
                                            self.publish_flag = False
                            
                                data_list = self.memProxy.getDataList("IsLookingAtRobot")
                                for i in range (len(data_list)):
                                    if data_list[i] == is_looking_at_robot_event_name:
                                        is_looking_at_robot = self.memProxy.getData(is_looking_at_robot_event_name)
                                        self.looking_at_robot.header.stamp = rospy.get_rostime()
                                        self.looking_at_robot.people = People_ID[0]
                                        self.looking_at_robot.is_looking_at_robot = is_looking_at_robot
                                        self.lookingAtRobotPub.publish(self.looking_at_robot)
                                    
                                    
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    gazeAnalysis = NaoqiGazeAnalysis()
    gazeAnalysis.start()
    rospy.spin()

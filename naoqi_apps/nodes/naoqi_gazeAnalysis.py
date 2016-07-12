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
from naoqi_driver.naoqi_node import NaoqiNode

class NaoqiGazeAnalysis (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_gazeAnalysis')
        self.connectNaoQi()

        self.gaze_direction = Float64MultiArray()
        self.head_angles = Float64MultiArray()
        self.gazeDirectionPub = rospy.Publisher("gaze_direction", Float64MultiArray, queue_size=10)
        self.headAnglesPub = rospy.Publisher("head_angles", Float64MultiArray, queue_size=10)        
        rospy.loginfo("naoqi_gazeAnalysis is initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                if self.memProxy.getData("ALBasicAwareness/HumanTracked") != -1:
                    People_ID = self.memProxy.getData("ALBasicAwareness/HumanTracked")
                    gaze_direction_event_name = "PeoplePerception/Person/" + str(People_ID) + "/GazeDirection"
                    head_angles_event_name = "PeoplePerception/Person/" + str(People_ID) + "/HeadAngles"
                   
                    if self.memProxy.getData(gaze_direction_event_name):
                        gaze_direction_array = self.memProxy.getData(gaze_direction_event_name) 
                        for i in range (len(gaze_direction_array)):
                            self.gaze_direction.data.append(gaze_direction_array[i]) 
                        if len(gaze_direction_array) != 0 :
                            self.gazeDirectionPub.publish(self.gaze_direction)
                            self.gaze_direction.data = []
                    
                    if self.memProxy.getData(gaze_direction_event_name):
                        head_angles_array = self.memProxy.getData(head_angles_event_name)
                        for i in range (len(head_angles_array)):
                            self.head_angles.data.append(head_angles_array[i])
                        if len(head_angles_array) != 0 :
                            self.headAnglesPub.publish(self.head_angles)
                            self.head_angles.data = []

            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    gazeAnalysis = NaoqiGazeAnalysis()
    gazeAnalysis.start()
    rospy.spin()

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
from std_msgs.msg import Bool
from naoqi_driver.naoqi_node import NaoqiNode

class NaoqiWavingDetection (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_wavingDetection')
        self.connectNaoQi()
        self.isWaving = Bool()
        self.isWavingCenter = Bool()
        self.isWavingLeft = Bool()
        self.isWavingRight = Bool()
        self.isWavingPub = rospy.Publisher("is_waving", Bool, queue_size=10)
        self.isWavingCenterPub = rospy.Publisher("is_waving_center", Bool, queue_size=10)
        self.isWavingRightPub = rospy.Publisher("is_waving_right", Bool, queue_size=10)
        self.isWavingLeftPub = rospy.Publisher("is_waving_left", Bool, queue_size=10)
        rospy.loginfo("naoqi_wavingDetection is initialized")
      
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        
        if self.memProxy is None:
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                data_list = self.memProxy.getDataList("ALBasicAwareness")
                for i in range (len(data_list)):
                    if data_list[i] == "ALBasicAwareness/HumanTracked":
                        People_ID = self.memProxy.getData("ALBasicAwareness/HumanTracked")
                        if People_ID != -1:
                            is_waving_event_name = "PeoplePerception/Person/" + str(People_ID) + "/IsWaving"
                            is_waving_center_event_name = "PeoplePerception/Person/" + str(People_ID) + "/IsWavingCenter"
                            is_waving_right_event_name = "PeoplePerception/Person/" + str(People_ID) + "/IsWavingRight"
                            is_waving_left_event_name = "PeoplePerception/Person/" + str(People_ID) + "/IsWavingLeft"
                            data_list = self.memProxy.getDataList("Person")
                            for i in range (len(data_list)):
                                if data_list[i] == is_waving_event_name:
                                    self.isWaving.data = self.memProxy.getData(is_waving_event_name) 
                                    self.isWavingPub.publish(self.isWaving)                                    
                   
                            for i in range (len(data_list)):
                                if data_list[i] == is_waving_right_event_name:
                                    self.isWavingRight.data = self.memProxy.getData(is_waving_right_event_name) 
                                    self.isWavingRightPub.publish(self.isWavingRight)                                    
                                
                            for i in range (len(data_list)):
                                if data_list[i] == is_waving_left_event_name:
                                    self.isWavingLeft.data = self.memProxy.getData(is_waving_left_event_name) 
                                    self.isWavingLeftPub.publish(self.isWavingLeft)                                    
                   
                            for i in range (len(data_list)):
                                if data_list[i] == is_waving_center_event_name:
                                    self.isWavingCenter.data = self.memProxy.getData(is_waving_center_event_name) 
                                    self.isWavingCenterPub.publish(self.isWavingCenter)                                    
                                    
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    wavingDetection = NaoqiWavingDetection()
    wavingDetection.start()
    rospy.spin()

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

class NaoqiFaceCharacteristics (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_faceCharacteristics')
        self.connectNaoQi()
        self.smile = Float64MultiArray()
        self.expression = Float64MultiArray()
        self.smilePub = rospy.Publisher("smile", Float64MultiArray, queue_size=10)
        self.expressionPub = rospy.Publisher("expression", Float64MultiArray, queue_size=10)        
        rospy.loginfo("naoqi_faceCharacteristics is initialized")
        self.pre_smile_array = [0.0, 0.0] # smile degree, confidence
        self.pre_expression_array = [0.0, 0.0, 0.0, 0.0, 0.0] # neutral, happy, surprised, angry or sad
        self.publish_flag = False

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.faceC = self.get_proxy("ALFaceCharacteristics")
        
        if self.memProxy is None:
            exit(1)
        if self.faceC is None:
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                data_list = self.memProxy.getDataList("ALBasicAwareness")
                for i in range (len(data_list)):
                    if data_list[i] == "ALBasicAwareness/HumanTracked":
                        People_ID = self.memProxy.getData("ALBasicAwareness/HumanTracked")
                        if People_ID != -1:
                            smile_event_name = "PeoplePerception/Person/" + str(People_ID) + "/SmileProperties"
                            expression_event_name = "PeoplePerception/Person/" + str(People_ID) + "/ExpressionProperties"
                            self.faceC.analyzeFaceCharacteristics(People_ID)
                            data_list = self.memProxy.getDataList("Person")
                            for i in range (len(data_list)):
                                if data_list[i] == smile_event_name:
                                    smile_array = self.memProxy.getData(smile_event_name) 
                                    if len(smile_array) > 0:
                                        for i in range (len(smile_array)):
                                            self.smile.data.append(smile_array[i]) 
                                        for i in range (len(smile_array)):
                                            if smile_array[i] != self.pre_smile_array[i]:
                                                self.publish_flag = True
                                        if self.publish_flag == True:
                                            self.smilePub.publish(self.smile)
                                        for i in range (len(smile_array)):
                                            self.pre_smile_array[i] = smile_array[i] 
                                        self.smile_array = []
                                        self.smile.data = []
                                        self.publish_flag = False
                                    
                                    
                            data_list = self.memProxy.getDataList("Person")
                            for i in range (len(data_list)):
                                if data_list[i] == expression_event_name:
                                    expression_array = self.memProxy.getData(expression_event_name)
                                    if len(expression_array) > 0:
                                        for i in range (len(expression_array)):
                                            self.expression.data.append(expression_array[i]) 
                                        for i in range (len(expression_array)):
                                            if expression_array[i] != self.pre_expression_array[i]:
                                                self.publish_flag = True
                                        if self.publish_flag == True:
                                            self.expressionPub.publish(self.expression)
                                        for i in range (len(expression_array)):
                                            self.pre_expression_array[i] = expression_array[i] 
                                        self.expression_array = []
                                        self.expression.data = []
                                        self.publish_flag = False
                    
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    faceCharacteristics = NaoqiFaceCharacteristics()
    faceCharacteristics.start()
    rospy.spin()

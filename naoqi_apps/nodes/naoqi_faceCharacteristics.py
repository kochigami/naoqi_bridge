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
from naoqi_bridge_msgs.srv import (
    AnalyzeFaceCharacteristicsResponse,
    AnalyzeFaceCharacteristics,
    GetSmilingThresholdResponse,
    GetSmilingThreshold,
    SetSmilingThresholdResponse,
    SetSmilingThreshold)

class NaoqiFaceCharacteristics (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_faceCharacteristics')
        self.connectNaoQi()
        # self.smile = Float64MultiArray()
        # self.expression = Float64MultiArray()
        # self.smilePub = rospy.Publisher("smile", Float64MultiArray, queue_size=10)
        # self.expressionPub = rospy.Publisher("expression", Float64MultiArray, queue_size=10)        
        # rospy.loginfo("naoqi_faceCharacteristics is initialized")
        # self.pre_smile_array = [0.0, 0.0] # smile degree, confidence
        # self.pre_expression_array = [0.0, 0.0, 0.0, 0.0, 0.0] # neutral, happy, surprised, angry or sad
        # self.publish_flag = False

        self.analyzeFaceCharacteristicsSrv = rospy.Service("analyze_face_characteristics", AnalyzeFaceCharacteristics, self.handleAnalyzeFaceCharacteristics)
        self.getSmilingThresholdSrv = rospy.Service("get_smiling_threshold", GetSmilingThreshold, self.handleGetSmilingThreshold)
        self.setSmilingThresholdSrv = rospy.Service("set_smiling_threshold", SetSmilingThreshold, self.handleSetSmilingThreshold)

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.faceC = self.get_proxy("ALFaceCharacteristics")
        if self.memProxy is None and self.faceC is None:
            exit(1)

    def handleAnalyzeFaceCharacteristics (self, req):
        res = AnalyzeFaceCharacteristicsResponse()
        res.success = False
        try:
            res.success = self.faceC.analyzeFaceCharacteristics(req.people_id)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
                return res

    def handleGetSmilingThreshold (self, req):
        try:
            res = GetSmilingThresholdResponse()
            res.threshold = self.faceC.getSmilingThreshold() 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
                return None

    def handleSetSmilingThreshold (self, req):
        res = SetSmilingThresholdResponse()
        res.success = False
        try:
            self.faceC.setSmilingThreshold(req.threshold)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
                return res

    def run(self):
        while self.is_looping():
            try:
                data_list = self.memProxy.getDataList("VisiblePeopleList")
                for i in range (len(data_list)):
                    if data_list[i] == "PeoplePerception/VisiblePeopleList":
                        People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                        if (len(People_ID)) > 0:
                            for i in range(len(People_ID)):
                                age = "PeoplePerception/Person/" + str(People_ID[i]) + "/AgeProperties"
                                expression = "PeoplePerception/Person/" + str(People_ID[i]) + "/ExpressionProperties"
                                facial_parts = "PeoplePerception/Person/" + str(People_ID[i]) + "/FacialPartsProperties"
                                gender = "PeoplePerception/Person/" + str(People_ID[i]) + "/GenderProperties"
                                smile = "PeoplePerception/Person/" + str(People_ID[i]) + "/SmileProperties"
                                data_list =  self.memProxy.getDataList(age)
                                if (len (data_list)) > 0:
                                    age_data = self.memProxy.getData(age)
                                    if age_data != None:
                                        #TODO#

                                data_list =  self.memProxy.getDataList(expression)
                                if (len (data_list)) > 0:
                                    expression_data = self.memProxy.getData(expression)
                                    if expression_data != None:
                                        #TODO#

                                data_list =  self.memProxy.getDataList(facial_parts)
                                if (len (data_list)) > 0:
                                    facial_parts_data = self.memProxy.getData(facial_parts)
                                    if facial_parts_data != None:
                                        #TODO#

                                data_list =  self.memProxy.getDataList(gender)
                                if (len (data_list)) > 0:
                                    gender_data = self.memProxy.getData(gender)
                                    if gender_data != None:
                                        #TODO#

                                data_list =  self.memProxy.getDataList(smile)
                                if (len (data_list)) > 0:
                                    smile_data = self.memProxy.getData(smile)
                                    if smile_data != None:
                                        #TODO#
                                
                            # data_list = self.memProxy.getDataList(str(People_ID[0]))
                           
                            # #print data_list
                            # for i in range (len(data_list)):
                            #     if data_list[i] == smile_event_name:
                            #         smile_array = self.memProxy.getData(smile_event_name) 
                            #         if len(smile_array) > 0:
                            #             for i in range (len(smile_array)):
                            #                 self.smile.data.append(smile_array[i]) 
                            #             for i in range (len(smile_array)):
                            #                 if smile_array[i] != self.pre_smile_array[i]:
                            #                     self.publish_flag = True
                            #             if self.publish_flag == True:
                            #                 self.smilePub.publish(self.smile)
                            #             for i in range (len(smile_array)):
                            #                 self.pre_smile_array[i] = smile_array[i] 
                            #             self.smile_array = []
                            #             self.smile.data = []
                            #             self.publish_flag = False
                                    
                                    
                            # data_list = self.memProxy.getDataList("Person")
                            # for i in range (len(data_list)):
                            #     if data_list[i] == expression_event_name:
                            #         expression_array = self.memProxy.getData(expression_event_name)
                            #         if len(expression_array) > 0:
                            #             for i in range (len(expression_array)):
                            #                 self.expression.data.append(expression_array[i]) 
                            #             for i in range (len(expression_array)):
                            #                 if expression_array[i] != self.pre_expression_array[i]:
                            #                     self.publish_flag = True
                            #             if self.publish_flag == True:
                            #                 self.expressionPub.publish(self.expression)
                            #             for i in range (len(expression_array)):
                            #                 self.pre_expression_array[i] = expression_array[i] 
                            #             self.expression_array = []
                            #             self.expression.data = []
                            #             self.publish_flag = False
                    
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    faceCharacteristics = NaoqiFaceCharacteristics()
    faceCharacteristics.start()
    rospy.spin()

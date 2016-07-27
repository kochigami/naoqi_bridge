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
from std_msgs.msg import Float64
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty)

class NaoqiSonar (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_sonar')
        self.connectNaoQi()
        self.front_sonar = Float64()
        self.back_sonar = Float64()
        self.frontSonarPub = rospy.Publisher("front_sonar", Float64, queue_size=10)
        self.backSonarPub = rospy.Publisher("back_sonar", Float64, queue_size=10)    
        
        # http://doc.aldebaran.com/2-4/naoqi/sensors/alsonar.html says we should subscribe ALSonar module to turn on sonar, but sonar seems to work without executing it
        self.startSonarSrv = rospy.Service("start_sonar", Empty, self.handleStartSonarSrv)
        self.stopSonarSrv = rospy.Service("stop_sonar", Empty, self.handleStopSonarSrv)
        rospy.loginfo("naoqi_sonar is initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.sonarProxy = self.get_proxy("ALSonar")
        
        if self.memProxy is None:
            exit(1)
        if self.sonarProxy is None:
            exit(1)

    def handleStartSonarSrv (self, req):
        try:
            self.sonarProxy.subscribe("naoqi_sonar_application")
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStopSonarSrv (self, req):
        try:
            self.sonarProxy.unsubscribe("naoqi_sonar_application")
            return EmptyResponse()

        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                # SonarLateralLeftDetected SonarLateralRightDetected SonarLeftDetected SonarLeftNothingDetected SonarMiddleDetected SonarNothingDetected SonarRightDetected SonarRightNothingDetected don't work on Pepper.
                self.front_sonar.data = self.memProxy.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")
                self.frontSonarPub.publish(self.front_sonar)
                self.back_sonar.data = self.memProxy.getData("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value")
                self.backSonarPub.publish(self.back_sonar)
                    
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    sonar = NaoqiSonar()
    sonar.start()
    rospy.spin()

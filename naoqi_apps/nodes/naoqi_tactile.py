#!/usr/bin/env python

#
# ROS node to read Nao's bumpers and tactile sensors
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2010 Stefan Osswald, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#                                                                             
#  Copyright 2017 Aldebaran                                                   
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
from naoqi_bridge_msgs.msg import HandTouch, HeadTouch, Bumper
from std_msgs.msg import Bool
from distutils.version import LooseVersion

class NaoqiTactile(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tactile')
        self.connectNaoQi()

        self.rate = rospy.Rate(10)
        
        # init. messages:
        self.headTactile = HeadTouch()
        self.handTactile = HandTouch()
        self.bumper = Bumper()
        self.headTactilePub = rospy.Publisher("head_touch", HeadTouch, queue_size=10)
        self.handTactilePub = rospy.Publisher("hand_touch", HandTouch, queue_size=10)
        self.bumperPub = rospy.Publisher("bumper", Bumper, queue_size=10)

        #try:
        #    footContact = self.memProxy.getData("footContact", 0)
        #except RuntimeError:
        #    footContact = None

        #if footContact is None:
        #    self.hasFootContactKey = False
        #    rospy.loginfo("Foot contact key is not present in ALMemory, will not publish to foot_contact topic.")
        #else:
        #    self.hasFootContactKey = True
        #    self.footContactPub = rospy.Publisher("foot_contact", Bool, latch=True, queue_size=10)
        #    self.footContactPub.publish(footContact > 0.0)

        # constants in HeadTouch and Bumper will not be available in callback functions
        # as they are executed in the parent broker context (i.e. on robot),
        # so they have to be copied to member variables
        # self.headTactileTouchFrontButton = HeadTouch.buttonFront;
        # self.headTactileTouchMiddleButton = HeadTouch.buttonMiddle;
        # self.headTactileTouchRearButton = HeadTouch.buttonRear;
        # self.handTactileTouchRightBack = HandTouch.RIGHT_BACK;
        # self.handTactileTouchRightLeft = HandTouch.RIGHT_LEFT;
        # self.handTactileTouchRightRight = HandTouch.RIGHT_RIGHT;
        # self.handTactileTouchLeftBack = HandTouch.LEFT_BACK;
        # self.handTactileTouchLeftLeft = HandTouch.LEFT_LEFT;
        # self.TactileTouchLeftRight = HandTouch.LEFT_RIGHT;
        # self.bumperRightButton = Bumper.right;
        # self.bumperLeftButton = Bumper.left;

        #self.subscribe()

        rospy.loginfo("nao_tactile initialized")

    def connectNaoQi(self):
        #rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.systemProxy = self.get_proxy("ALSystem")
        if self.systemProxy is None:
            rospy.logerr("Could not get a proxy to ALSystem")
            exit(1)
        else:
            if LooseVersion(self.systemProxy.systemVersion()) < LooseVersion("2.4"):
                rospy.logerr("Naoqi version of your robot is " + str(self.systemProxy.systemVersion()) + ", which doesn't have a proxy to ALTactile.")
                exit(1)
            else:
                self.memProxy = self.get_proxy("ALMemory")
                if self.memProxy is None:
                    rospy.logerr("Could not get a proxy to ALMemory.")
                    exit(1)

    def run(self):
        while self.is_looping():
            try:
                if self.headTactilePub.get_num_connections() > 0:
                    front_head_touch = self.memProxy.getData("FrontTactilTouched")
                    head_touch = HeadTouch()
                    head_touch.button = 1
                    head_touch.state = int(front_head_touch)
                    self.headTactilePub.publish(head_touch)

                    middle_head_touch = self.memProxy.getData("MiddleTactilTouched")
                    head_touch = HeadTouch()
                    head_touch.button = 2
                    head_touch.state = int(front_head_touch)
                    self.headTactilePub.publish(head_touch)
                    
                    rear_head_touch = self.memProxy.getData("RearTactilTouched")
                    head_touch = HeadTouch()
                    head_touch.button = 3
                    head_touch.state = int(front_head_touch)
                    self.headTactilePub.publish(head_touch)

                elif self.handTactilePub.get_num_connections() > 0:
                    hand_right_back_touched = self.memProxy.getData("HandRightBackTouched")
                    if hand_right_back_touched:
                        hand_touch = HandTouch()
                        hand_touch.hand = 0
                        hand_touch.state = int(hand_right_back_touched)
                        self.handTactilePub.publish(hand_touch)

                    hand_right_left_touched = self.memProxy.getData("HandRightLeftTouched")
                    if hand_right_left_touched:
                        hand_touch = HandTouch()
                        hand_touch.hand = 1
                        hand_touch.state = int(hand_right_left_touched)
                        self.handTactilePub.publish(hand_touch)

                    hand_right_right_touched = self.memProxy.getData("HandRightRightTouched")
                    if hand_right_right_touched: 
                        hand_touch = HandTouch()
                        hand_touch.hand = 2
                        hand_touch.state = int(hand_right_right_touched)
                        self.handTactilePub.publish(hand_touch)
                    
                    hand_left_back_touched = self.memProxy.getData("HandLeftBackTouched")
                    if hand_left_back_touched:
                        hand_touch = HandTouch()
                        hand_touch.hand = 3
                        hand_touch.state = int(hand_left_back_touched)
                        self.handTactilePub.publish(hand_touch)

                    hand_left_left_touched = self.memProxy.getData("HandLeftLeftTouched")
                    if hand_left_left_touched:
                        hand_touch = HandTouch()
                        hand_touch.hand = 4
                        hand_touch.state = int(hand_left_left_touched)
                        self.handTactilePub.publish(hand_touch)
                        
                    hand_left_right_touched = self.memProxy.getData("HandLeftRightTouched")
                    if hand_left_right_touched:
                        hand_touch = HandTouch()
                        hand_touch.hand = 5
                        hand_touch.state = int(hand_left_right_touched)
                        self.handTactilePub.publish(hand_touch)

                elif self.bumperPub.get_num_connections() > 0:
                    right_bumper_pressed = self.memProxy.getData("RightBumperPressed")
                    bumper = Bumper()
                    bumper.bumper = 0
                    bumper.state = int(right_bumper_pressed)
                    
                    left_bumper_pressed = self.memProxy.getData("LeftBumperPressed")
                    bumper = Bumper()
                    bumper.bumper = 1
                    bumper.state = int(left_bumper_pressed)

            except RuntimeError as e:
                print("Error accessing ALMemory, exiting...\n")
                print(e)
                rospy.signal_shutdown("No NaoQI available anymore")

            self.rate.sleep()
                    

                    
if __name__ == '__main__':
    tactile = NaoqiTactile()
    tactile.start()
    rospy.spin()


    #     if self.hasFootContactKey:
    #         self.memProxy.subscribeToEvent("footContactChanged", self.moduleName, "onFootContactChanged")


    # def unsubscribe(self):
    #     self.memProxy.unsubscribeToEvent("FrontTactilTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("MiddleTactilTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("RearTactilTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("RightBumperPressed", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("LeftBumperPressed", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandLeftBackTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandLeftLeftTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandLeftRightTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandRightBackTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandRightLeftTouched", self.moduleName)
    #     self.memProxy.unsubscribeToEvent("HandRightRightTouched", self.moduleName)
    #     if self.hasFootContactKey:
    #         self.memProxy.unsubscribeToEvent("footContactChanged", self.moduleName)


    # def onTactileChanged(self, strVarName, value, strMessage):
    #     "Called when tactile touch status changes in ALMemory"
    #     if strVarName == "FrontTactilTouched":
    #         self.headTactile.button = self.headTactileTouchFrontButton
    #     elif strVarName == "MiddleTactilTouched":
    #         self.headTactile.button = self.headTactileTouchMiddleButton
    #     elif strVarName == "RearTactilTouched":
    #         self.headTactile.button = self.headTactileTouchRearButton
    #     elif strVarName == "HandLeftBackTouched":
    #         self.handTactile.hand = self.handTactileTouchRightBack
    #     elif strVarName == "HandLeftLeftTouched":
    #         self.handTactile.hand = self.handTactileTouchRightLeft
    #     elif strVarName == "HandLeftRightTouched":
    #         self.handTactile.hand = self.handTactileTouchRightRight
    #     elif strVarName == "HandRightBackTouched":
    #         self.handTactile.hand = self.handTactileTouchLeftBack
    #     elif strVarName == "HandRightLeftTouched":
    #         self.handTactile.hand = self.handTactileTouchLeftLeft
    #     elif strVarName == "HandRightRightTouched":
    #         self.handTactile.hand = self.handTactileTouchLeftRight

    #     self.handTactile.state = int(value);
    #     self.headTactile.state = int(value);
    #     self.headTactilePub.publish(self.headTactile)
    #     self.handTactilePub.publish(self.handTactile)
    #     rospy.logdebug("tactile touched: name=%s, value=%d, message=%s.", strVarName, value, strMessage);

    # def onBumperChanged(self, strVarName, value, strMessage):
    #     #"Called when bumper status changes in ALMemory"
    #     if strVarName == "RightBumperPressed":
    #         self.bumper.bumper = self.bumperRightButton
    #     elif strVarName == "LeftBumperPressed":
    #         self.bumper.bumper = self.bumperLeftButton

    #     self.bumper.state = int(value);
    #     self.bumperPub.publish(self.bumper)
    #     rospy.logdebug("bumper pressed: name=%s, value=%d, message=%s.", strVarName, value, strMessage);

    # def onFootContactChanged(self, strVarName, value, strMessage):
    #     #"Called when foot contact changes in ALMemory"
    #     self.footContactPub.publish(value > 0.0)

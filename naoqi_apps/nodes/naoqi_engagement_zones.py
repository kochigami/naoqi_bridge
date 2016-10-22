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
from naoqi_driver.naoqi_node import NaoqiNode
# from naoqi_bridge_msgs.srv import ( 
#     SetArmsEnabledResponse,
#     SetArmsEnabled,
#     MoveIsActiveResponse,
#     MoveIsActive,
#     GetStiffnessesResponse,
#     GetStiffnesses,
#     SetStiffnessesResponse,
#     SetStiffnesses,
#     GetRobotPositionResponse,
#     GetRobotPosition,)

class NaoqiEngagementZones(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_engagement_zones')
        self.connectNaoQi()
        
        self.computeEngagementZoneSrv = rospy.Service("compute_engagement_zone", TODO, self.handleComputeEngagementZoneSrv)
        self.getFirstLimitDistanceSrv = rospy.Service("get_first_limit_distance", TODO, self.handleGetFirstLimitDistanceSrv)
        self.getLimitAngleSrv = rospy.Service("get_limit_angle", TODO, self.handleGetLimitAngleSrv)
        self.getSecondLimitDistanceSrv = rospy.Service("get_second_limit_distance", TODO, self.handleGetSecondLimitDistanceSrv)
        self.setFirstLimitDistanceSrv = rospy.Service("set_first_limit_distance", TODO, self.handleSetFirstLimitDistanceSrv)
        self.setLimitAngleSrv = rospy.Service("set_limit_angle", TODO, self.handleSetLimitAngleSrv)
        self.setSecondLimitDistanceSrv = rospy.Service("set_second_limit_distance", TODO, self.handleSetSecondLimitDistanceSrv)
        self.limitAngleUpdatedPub = rospy.Publisher("limit_angle_updated", TODO, queue_size=10)
        self.firstLimitDistanceUpdatedPub = rospy.Publisher("first_limit_distance_updated", TODO, queue_size=10)
        self.secondLimitDistanceUpdatedPub = rospy.Publisher("second_limit_distance_updated", TODO, queue_size=10)
        self.peopleInZonesUpdatedPub = rospy.Publisher("people_in_zones_updated", TODO, queue_size=10)
        self.personApproachedPub = rospy.Publisher("person_approached", TODO, queue_size=10)
        self.personMovedAwayPub = rospy.Publisher("person_moved_away", TODO, queue_size=10)
        self.personEnteredZone1Pub = rospy.Publisher("person_entered_zone1", TODO, queue_size=10)
        self.personEnteredZone2Pub = rospy.Publisher("person_entered_zone2", TODO, queue_size=10)
        self.personEnteredZone3Pub = rospy.Publisher("person_entered_zone3", TODO, queue_size=10)
        self.movementInZonesUpdatedPub = rospy.Publisher("movement_in_zone_updated", TODO, queue_size=10)
        self.engagementZoneByPersonPub = rospy.Publisher("engagement_zone_by_person", TODO, queue_size=10)
        self.peopleInZone1Pub = rospy.Publisher("people_in_zone1", TODO, queue_size=10)
        self.peopleInZone2Pub = rospy.Publisher("people_in_zone2", TODO, queue_size=10)
        self.peopleInZone3Pub = rospy.Publisher("people_in_zone3", TODO, queue_size=10)
        self.lastMovementsInZone1Pub = rospy.Publisher("last_movements_in_zone1", TODO, queue_size=10)
        self.lastMovementsInZone2Pub = rospy.Publisher("last_movements_in_zone2", TODO, queue_size=10)
        self.lastMovementsInZone3Pub = rospy.Publisher("last_movements_in_zone3", TODO, queue_size=10) 
       rospy.loginfo("naoqi_engagement_zones initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.engagementProxy = self.get_proxy("ALEngagementZones")
        self.peopleProxy = self.get_proxy("ALPeoplePerception")
        self.moveDetectionProxy = self.get_proxy("ALMoveDetection")
        if self.memProxy is None or self.engagementProxy is None or self.peopleProxy is None or self.moveDetectionProxy is None:
            exit(1)
    
    def handleSetMoveArmsEnabledSrv(self, req):
        try:
            self.motionProxy.setMoveArmsEnabled(req.left_arm, req.right_arm)
            rospy.loginfo("Left Arm is set to " + str(req.left_arm) + "and Right Arm is set to " + str(req.right_arm))
            return SetArmsEnabledResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    engagement_zones = NaoqiEngagementZones()
    engagement_zones.start()
    rospy.spin()

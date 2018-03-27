#!/usr/bin/env python

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Vector3

from pioneer_mrs.msg import MissionState
from pioneer_mrs.msg import CommunicationState
from pioneer_mrs.srv import *

class Algorithm:
    def __init__(self):
        self._hostname = rospy.get_param('~hostname', 'robot1')
        self._host_num = int(self._hostname[-1])
        self._host_index = self._host_num - 1
        self._Q = 0.2
        self._state = False
        self._formation_offset = 0.0
        self._comm_state = [False, False, False, False, False]
        self._pose_offset = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._gradient = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._team_pose = np.array([(0,0), (0,0), (0,0), (0,0), (0,0)], dtype=np.float32)
        self._my_pose = np.array((0,0), dtype=np.float32)
        

        rospy.Subscriber("mission_state", MissionState, self.mission_state_callback)
        rospy.Subscriber("comm_state", CommunicationState, self.communication_state_callback)
        self.vel_hp_pub = rospy.Publisher("cmd_vel_hp", Vector3, queue_size=1)
        
        for i in range(5):
            rospy.loginfo('loop i=' + str(i))
            rospy.wait_for_service("/robot" + str(i+1) + "/get_pose")
        get_pose_1 = rospy.ServiceProxy("/robot1/get_pose", Pose2D, persistent=True)
        get_pose_2 = rospy.ServiceProxy("/robot2/get_pose", Pose2D, persistent=True)
        get_pose_3 = rospy.ServiceProxy("/robot3/get_pose", Pose2D, persistent=True)
        get_pose_4 = rospy.ServiceProxy("/robot4/get_pose", Pose2D, persistent=True)
        get_pose_5 = rospy.ServiceProxy("/robot5/get_pose", Pose2D, persistent=True)
        self.get_pose = (get_pose_1, get_pose_2, get_pose_3, get_pose_4, get_pose_5)

        rospy.loginfo(self._hostname + ' algorithm node launched.')

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.run_algorithm()
            rate.sleep()


    def mission_state_callback(self, msg):
        self._state = msg.algorithm
        self._formation_offset = msg.formation_offset

    def communication_state_callback(self, msg):
        for i in range(5):
            self._comm_state[i] = msg.state[i]

    def update_team_pose(self):
        for i in range(5):
            srv = self.get_pose[i]()
            self._team_pose[i][0] = srv.x  # convert srv type to numpy type
            self._team_pose[i][1] = srv.y
        self._my_pose = self._team_pose[self._host_index]


    def run_algorithm(self):
        if self._state:
            self.update_team_pose()

            team_pose_offset = self._team_pose - self._pose_offset
            my_pose_offset = self._my_pose - self._pose_offset[self._host_index]

            vel_sum = np.array((0,0), dtype=np.float32)
            formation_offset = np.array((self._formation_offset ,0), dtype=np.float32)
            for i in range(5):
                if self._comm_state[i] and self._team_pose[i].any() != 0:
                    vel_sum += (team_pose_offset[i] - my_pose_offset) * self._Q
            rospy.loginfo(self._hostname + " vel_sum " + str(vel_sum) )

            vel_sum = vel_sum - (my_pose_offset - self._gradient[self._host_index] - formation_offset)

            vel = Vector3()
            vel.x = vel_sum[0]
            vel.y = vel_sum[1]
            rospy.loginfo(self._hostname + " pub vel [x=" + str(vel.x) + " y=" + str(vel.y) + "]" )
            self.vel_hp_pub.publish(vel)


if __name__ == '__main__':
    rospy.init_node('algorithm_node')
    try:
        algorithm = Algorithm()
    except rospy.ROSInterruptException:
        pass
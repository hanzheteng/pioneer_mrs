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
        # robot parameters
        self._hostname = rospy.get_param('~hostname', 'robot1')
        self._host_num = int(self._hostname[-1])
        self._host_index = self._host_num - 1
        self._state = False
        self._comm_state = [False, False, False, False, False]

        # algorithm parameters
        self._gain_q = 0.2
        self._moving_distance = 0.0
        self._pose_offset = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._gradient    = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._team_pose   = np.array([(0,0), (0,0), (0,0), (0,0), (0,0)], dtype=np.float32)
        self._my_pose     = np.array((0,0), dtype=np.float32)

        # sub and pub init
        rospy.Subscriber("mission_state", MissionState, self.mission_state_callback)
        rospy.Subscriber("comm_state", CommunicationState, self.communication_state_callback)
        self.vel_hp_pub = rospy.Publisher("cmd_vel_hp", Vector3, queue_size=1)

        # service client init
        for i in range(5):
            rospy.wait_for_service("/robot" + str(i+1) + "/get_pose")
        get_pose_1 = rospy.ServiceProxy("/robot1/get_pose", Pose2D, persistent=True)
        get_pose_2 = rospy.ServiceProxy("/robot2/get_pose", Pose2D, persistent=True)
        get_pose_3 = rospy.ServiceProxy("/robot3/get_pose", Pose2D, persistent=True)
        get_pose_4 = rospy.ServiceProxy("/robot4/get_pose", Pose2D, persistent=True)
        get_pose_5 = rospy.ServiceProxy("/robot5/get_pose", Pose2D, persistent=True)
        self.get_pose = (get_pose_1, get_pose_2, get_pose_3, get_pose_4, get_pose_5)

        # done
        rospy.loginfo(self._hostname + ' algorithm_node launched.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.run_algorithm()
            rate.sleep()


    def mission_state_callback(self, msg):
        self._state = msg.algorithm
        self._moving_distance = msg.moving_distance


    def communication_state_callback(self, msg):
        for i in range(5):
            self._comm_state[i] = msg.state[i]


    def update_team_pose(self):
        for i in range(5):
            # call get_pose service
            srv = self.get_pose[i]()
            # convert srv type to numpy type
            self._team_pose[i][0] = srv.x
            self._team_pose[i][1] = srv.y
        # get my pose
        self._my_pose = self._team_pose[self._host_index]


    def run_algorithm(self):
        # check flag to start or stop
        if self._state:
            # make a log for communication time
            rospy.loginfo(self._hostname + " algorithm_node: Updating team pose.")
            self.update_team_pose()
            rospy.loginfo(self._hostname + " algorithm_node: Updated team pose.")

            # prepare local variables (add offset to prevent robots from crashing into the same consensus point)
            team_pose_with_offset = self._team_pose - self._pose_offset
            my_pose_with_offset   = self._my_pose   - self._pose_offset[self._host_index]
            vel_sum               = np.array((0,0), dtype=np.float32)
            moving_distance       = np.array((self._moving_distance ,0), dtype=np.float32)

            # consensus algorithm makes it keep formation
            for i in range(5):
                # check communication graph and the correctness of pose data
                if self._comm_state[i] and self._team_pose[i].any() != 0:
                    vel = (team_pose_with_offset[i] - my_pose_with_offset) * self._gain_q
                    vel_sum = vel_sum + vel
                    rospy.loginfo(self._hostname + " algorithm_node: neighbor pose (x=%.5f,y=%.5f)", \
                        self._team_pose[i][0], self._team_pose[i][1])
                    rospy.loginfo(self._hostname + " algorithm_node: vel (Vx=%.5f,Vy=%.5f)", vel[0], vel[1])

            # gradient descent makes the consensus point stick to the optimal point
            gradient = my_pose_with_offset - self._gradient[self._host_index]
            vel_sum = vel_sum - (gradient - moving_distance)
            rospy.loginfo(self._hostname + " algorithm_node: gradient (x=%.5f,y=%.5f)", gradient[0], gradient[1])

            # publish velocity result (vel of hand point)
            vel_hp = Vector3()
            vel_hp.x = vel_sum[0]
            vel_hp.y = vel_sum[1]
            self.vel_hp_pub.publish(vel_hp)
            rospy.loginfo(self._hostname + " algorithm_node: pub vel_hp (x=%.5f,y=%.5f)", vel_hp.x, vel_hp.y)


if __name__ == '__main__':
    rospy.init_node('algorithm_node')
    try:
        algorithm = Algorithm()
    except rospy.ROSInterruptException:
        pass

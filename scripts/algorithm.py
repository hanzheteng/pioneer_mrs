#!/usr/bin/env python

import rospy
import sys
import numpy as np
import actionlib

from geometry_msgs.msg import Vector3

from pioneer_mrs.msg import *  # include msg for action
from pioneer_mrs.srv import *

class Algorithm:
    def __init__(self):
        # robot parameters
        self._hostname = rospy.get_param('~hostname', 'robot1')
        self._hostnum = int(self._hostname[-1]) - 1  # index 0 to 4
        self._comm_state = [False, False, False, False, False]

        # algorithm parameters
        self._gain_q = 0.2
        self._moving_distance = 0.0
        self._pose_offset = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._gradient    = np.array([(-1,1), (1,1), (0,0), (1,-1), (-1,-1)], dtype=np.float32)
        self._team_pose   = np.array([(0,0), (0,0), (0,0), (0,0), (0,0)], dtype=np.float32)
        self._my_pose     = np.array((0,0), dtype=np.float32)

        # sub and pub init
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

        # action server init
        self._feedback = pioneer_mrs.msg.FormationFeedback()
        self._result = pioneer_mrs.msg.FormationResult()
        self._action_name = "Formation"
        self._as = actionlib.SimpleActionServer(self._action_name, pioneer_mrs.msg.FormationAction, \
            execute_cb=self.action_callback, auto_start=False)
        self._as.start()

        # done
        rospy.loginfo(self._hostname + " algorithm_node launched.")
        rospy.spin()


    def communication_state_callback(self, msg):
        for i in range(5):
            self._comm_state[i] = msg.state[i]


    def action_callback(self, goal):
        # action started
        success = True
        rospy.loginfo(self._hostname + " algorithm_node action started")

        # run action for goal.order steps
        rate = rospy.Rate(10)
        for step in range(1, goal.order):
            # send feedback and check preempt every second
            if step % 10 == 0:
                self._feedback.name = self._hostname
                self._feedback.x = self._my_pose[0]
                self._feedback.y = self._my_pose[1]
                self._as.publish_feedback(self._feedback)
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    rospy.logerr(self._hostname + " algorithm_node action preempted")
                    break
            # run algorithm
            self.run_algorithm()
            rate.sleep()

        # action succeeded
        if success:
            self._result.name = self._hostname
            self._result.x = self._my_pose[0]
            self._result.y = self._my_pose[1]
            self._as.set_succeeded(self._result)
            rospy.loginfo(self._hostname + " algorithm_node action succeeded")


    def update_team_pose(self):
        for i in range(5):
            # call get_pose service
            srv = self.get_pose[i]()
            # convert srv type to numpy type
            self._team_pose[i][0] = srv.x
            self._team_pose[i][1] = srv.y
        # get my pose
        self._my_pose = self._team_pose[self._hostnum]


    def run_algorithm(self):
        # make a log for communication time
        rospy.loginfo(self._hostname + " algorithm_node: Updating team pose.")
        self.update_team_pose()
        rospy.loginfo(self._hostname + " algorithm_node: Updated team pose.")

        # prepare local variables (add offset to prevent robots from crashing into the same consensus point)
        team_pose_with_offset = self._team_pose - self._pose_offset
        my_pose_with_offset   = self._my_pose   - self._pose_offset[self._hostnum]
        vel_sum               = np.array((0,0), dtype=np.float32)
        moving_distance       = np.array((self._moving_distance ,0), dtype=np.float32)

        # consensus algorithm makes it keep formation
        for i in range(5):
            # check communication graph and the correctness of pose data
            if self._comm_state[i] and self._team_pose[i].any() != 0:
                vel = (team_pose_with_offset[i] - my_pose_with_offset)
                vel_sum = vel_sum + vel
                rospy.loginfo(self._hostname + " algorithm_node: robot" + str(i+1) + \
                    " pose (% .4f,% .4f)", self._team_pose[i][0], self._team_pose[i][1])
                rospy.loginfo(self._hostname + " algorithm_node: vel" + str(i+1) + \
                    " (% .4f,% .4f)", vel[0], vel[1])

        # gradient descent makes the consensus point stick to the optimal point
        gradient = my_pose_with_offset - self._gradient[self._hostnum]
        vel_sum = vel_sum - (gradient - moving_distance) / np.sqrt(self._gain_q)
        rospy.loginfo(self._hostname + " algorithm_node: gradient (% .4f,% .4f)", gradient[0], gradient[1])

        # update gain q
        self._gain_q = self._gain_q + np.arctan(np.exp(np.linalg.norm( my_pose_with_offset )))
        rospy.loginfo(self._hostname + " algorithm_node: gain q = %.2f ", self._gain_q)

        # publish velocity result (vel of hand point)
        vel_hp = Vector3()
        vel_hp.x = vel_sum[0]
        vel_hp.y = vel_sum[1]
        self.vel_hp_pub.publish(vel_hp)
        rospy.loginfo(self._hostname + " algorithm_node: pub vel_hp (% .4f,% .4f)", vel_hp.x, vel_hp.y)


if __name__ == '__main__':
    rospy.init_node('algorithm_node')
    try:
        algorithm = Algorithm()
    except rospy.ROSInterruptException:
        pass

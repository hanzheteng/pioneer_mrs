#!/usr/bin/env python
import sys
import termios
import tty
import time
import rospy
import actionlib

from geometry_msgs.msg import Vector3
from pioneer_mrs.msg import *

welcomeMessage = """
Welcome to the commander of this multi-robot system.
    Press number 1-5 to switch the robot.
    Press 9 to select all and 0 to select none.
For selected robots:
    Press 'arrow keys' to move the robot.
    Press 'space bar' to stop the robot.
    Press 'm' to start a multi-robot algorithm action.
    Press 's' to stop the action.
"""

robotList = ('0', '1', '2', '3', '4', '5', '9')
robotSwitch = [False, False, False, False, False]

# up, down, right, left, stop
movementList = {'\x1b[A':(1,0), '\x1b[B':(-1,0), '\x1b[C':(0,-1), '\x1b[D':(0,1), ' ':(0,0)}

# multi-robot algorithm, stop action
missionList = ('m', 's')


def getKey():
    """ get one character from the terminal """
    fd = sys.stdin.fileno()  # file descriptor of stdin
    old_settings = termios.tcgetattr(fd)  # get attributes of terminal
    try:
        tty.setraw(fd)  # set terminal mode to raw
        key = sys.stdin.read(1)  # read one character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) # set to old settings
    return key


def teleoperation():
    """ get commmands from the keyboard and send to robots """
    pub_vel_1 = rospy.Publisher('/robot1/cmd_vel_hp', Vector3, queue_size=1)
    pub_vel_2 = rospy.Publisher('/robot2/cmd_vel_hp', Vector3, queue_size=1)
    pub_vel_3 = rospy.Publisher('/robot3/cmd_vel_hp', Vector3, queue_size=1)
    pub_vel_4 = rospy.Publisher('/robot4/cmd_vel_hp', Vector3, queue_size=1)
    pub_vel_5 = rospy.Publisher('/robot5/cmd_vel_hp', Vector3, queue_size=1)
    pub_vel = (pub_vel_1, pub_vel_2, pub_vel_3, pub_vel_4, pub_vel_5)

    client_1 = actionlib.SimpleActionClient('/robot1/Formation', pioneer_mrs.msg.FormationAction)
    client_2 = actionlib.SimpleActionClient('/robot2/Formation', pioneer_mrs.msg.FormationAction)
    client_3 = actionlib.SimpleActionClient('/robot3/Formation', pioneer_mrs.msg.FormationAction)
    client_4 = actionlib.SimpleActionClient('/robot4/Formation', pioneer_mrs.msg.FormationAction)
    client_5 = actionlib.SimpleActionClient('/robot5/Formation', pioneer_mrs.msg.FormationAction)
    client = (client_1, client_2, client_3, client_4, client_5)
    goal = pioneer_mrs.msg.FormationGoal(order=100)
    for i in range(5):
        client[i].wait_for_server()

    print welcomeMessage

    while not rospy.is_shutdown():
        key = getKey()
        if key == '\x1b':
            key += getKey()
            key += getKey()

        if key in robotList:
            if key == '0':
                for i in range(5):
                    robotSwitch[i] = False
            elif key == '9':
                for i in range(5):
                    robotSwitch[i] = True
            else:
                robotSwitch[int(key)-1] = not robotSwitch[int(key)-1]
            rospy.loginfo("robots:" + str(robotSwitch))

        elif key in movementList:
            vel = Vector3()  # default zero
            vel.x = movementList[key][0] * 0.4
            vel.y = movementList[key][1] * 0.4
            for i in range(5):
                if robotSwitch[i]:
                    pub_vel[i].publish(vel)

        elif key in missionList:
            if key == 'm':
                for i in range(5):
                    client[i].send_goal(goal, feedback_cb=action_feedback_cb, done_cb=action_done_cb)
                rospy.loginfo("start a multi-robot algorithm action")
                for i in range(5):
                    client[i].wait_for_result()
                time.sleep(1)
                print welcomeMessage
            else:
                for i in range(5):
                    client[i].cancel_all_goals()
                rospy.loginfo("stop the action")
                time.sleep(1)
                print welcomeMessage

        elif key == '\x03':
            break


def action_feedback_cb(feedback):
    rospy.loginfo(feedback.name + " (%.5f,%.5f) ", feedback.x, feedback.y)

def action_done_cb(success, result):
    rospy.loginfo(result.name + " result (%.5f,%.5f) ", result.x, result.y)


if __name__ == "__main__":
    rospy.init_node('teleop')
    try:
        teleoperation()
    except rospy.ROSInterruptException:
        pass

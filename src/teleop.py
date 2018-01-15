#!/usr/bin/env python
import sys
import termios
import tty
import rospy

from geometry_msgs.msg import Vector3
from pioneer_mrs.msg import MissionState

welcomeMessage = """
Welcome to be the commander of this multi-robot system.
Press number 1-5 to switch the robot.
Press 9 to select all and 0 to select none.
For selected robots:
    Press 'arrow keys' to move the robot.
    Press 'space bar' to stop the robot.
    Press 't' to start a trajectory mission.
    Press 'm' to start a multi-robot algorithm mission.
    Press 's' to stop the mission.
"""

robotList = ('0', '1', '2', '3', '4', '5', '9')
robotSwitch = [False, False, False, False, False]

# up, down, right, left, stop
movementList = {'\x1b[A':(1,0), '\x1b[B':(-1,0), '\x1b[C':(0,-1), '\x1b[D':(0,1), ' ':(0,0)}

# trajectory, multi-robot algorithm, stop
missionList = ('t', 'm', 's')


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
    pub_vel = [pub_vel_1, pub_vel_2, pub_vel_3, pub_vel_4, pub_vel_5]

    pub_cmd_1 = rospy.Publisher('/robot1/mission_state', MissionState, queue_size=1)
    pub_cmd_2 = rospy.Publisher('/robot2/mission_state', MissionState, queue_size=1)
    pub_cmd_3 = rospy.Publisher('/robot3/mission_state', MissionState, queue_size=1)
    pub_cmd_4 = rospy.Publisher('/robot4/mission_state', MissionState, queue_size=1)
    pub_cmd_5 = rospy.Publisher('/robot5/mission_state', MissionState, queue_size=1)
    pub_cmd = [pub_cmd_1, pub_cmd_2, pub_cmd_3, pub_cmd_4, pub_cmd_5]

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
            rospy.loginfo("robots:" + str(robotSwitch) )

        elif key in movementList:
            vel = Vector3()  # default zero
            vel.x = movementList[key][0] * 0.2
            vel.y = movementList[key][1] * 0.2
            for i in range(5):
                if robotSwitch[i]:
                    pub_vel[i].publish(vel)

        elif key in missionList:
            cmd = MissionState()  # default False
            if key == 't':
                cmd.trajectory = True
                rospy.loginfo("start a trajectory mission")
            elif key == 'm':
                cmd.algorithm = True
                rospy.loginfo("start a multi-robot algorithm mission")
            else:
                rospy.loginfo("stop the mission")
            for i in range(5):
                if robotSwitch[i]:
                    pub_cmd[i].publish(cmd)

        elif key == '\x03':
            break

if __name__ == "__main__":
    rospy.init_node('teleop')
    try:
        teleoperation()
    except rospy.ROSInterruptException:
        pass

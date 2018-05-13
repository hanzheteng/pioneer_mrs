#!/usr/bin/env python
import sys
import termios
import tty
import commands
import subprocess as sp
import time
import rospy
import actionlib

from geometry_msgs.msg import Vector3
from pioneer_mrs.msg import *

class Teleop:
    def __init__(self):
        # prompt message to user
        self._welcome_message = """
Welcome to the commander of this multi-robot system.
    Press number 1-5 to switch the robot.
    Press 9 to select all and 0 to select none.
For selected robots:
    Press 'arrow keys' to move the robot.
    Press 'space bar' to stop the robot.
    Press 'm' to start a multi-robot algorithm action.
    Press 's' to stop the action.
"""
        # none, robot1 - robot5, all
        self._robot_list = ('0', '1', '2', '3', '4', '5', '9')

        # control status for give robots
        self._robot_switch = [False, False, False, False, False]

        # up, down, right, left, stop
        self._movement_list = {'\x1b[A':(1,0), '\x1b[B':(-1,0), '\x1b[C':(0,-1), '\x1b[D':(0,1), ' ':(0,0)}

        # multi-robot algorithm, stop action
        self._mission_list = ('m', 's')

        # position info for five robots
        self._robot_position = [0, [0,0], [0,0], [0,0], [0,0], [0,0]]

        # run main loop
        self.teleoperation()


    def get_key(self):
        """ get one character from the terminal """
        fd = sys.stdin.fileno()  # file descriptor of stdin
        old_settings = termios.tcgetattr(fd)  # get attributes of terminal
        try:
            tty.setraw(fd)  # set terminal mode to raw
            key = sys.stdin.read(1)  # read one character
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) # set to old settings
        return key


    def redirect_screen_output(self, status):
        if status:
            # redirect screen output to an xterm terminal
            old_tty = commands.getoutput("ls /dev/pts").split('\n')
            self.xterm = sp.Popen(["xterm"])
            time.sleep(1)
            new_tty = commands.getoutput("ls /dev/pts").split('\n')
            num = list(set(new_tty).difference(set(old_tty)))[0]
            self.old_stdout = sys.stdout
            self.fd = open("/dev/pts/"+str(num), 'w')
            sys.stdout = self.fd
            print "\n"
        else:
            # restore stdout
            self.xterm.kill()
            sys.stdout = self.old_stdout
            self.fd.close()


    def teleoperation(self):
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

        self.plot_x = rospy.Publisher('plot/x', PointArray, queue_size=10)
        self.plot_y = rospy.Publisher('plot/y', PointArray, queue_size=10)
        
        for i in range(5):
            client[i].wait_for_server()

        print self._welcome_message

        #self.redirect_screen_output(True)

        # main loop
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == '\x1b':   # arrow key has three char
                key += self.get_key()
                key += self.get_key()

            if key in self._robot_list:
                if key == '0':
                    for i in range(5):
                        self._robot_switch[i] = False
                elif key == '9':
                    for i in range(5):
                        self._robot_switch[i] = True
                else:
                    self._robot_switch[int(key)-1] = not self._robot_switch[int(key)-1]
                rospy.loginfo("robots:" + str(self._robot_switch))

            elif key in self._movement_list:
                vel = Vector3()  # default zero
                vel.x = self._movement_list[key][0] * 0.4   # set vel to 0.4 m/s
                vel.y = self._movement_list[key][1] * 0.4
                for i in range(5):
                    if self._robot_switch[i]:
                        pub_vel[i].publish(vel)

            elif key in self._mission_list:
                if key == 'm':
                    for i in range(5):
                        client[i].send_goal(goal, feedback_cb=self.action_feedback_cb, \
                        done_cb=self.action_done_cb)
                    rospy.loginfo("start a multi-robot algorithm action")
                    for i in range(5):
                        client[i].wait_for_result()
                    time.sleep(1)
                    print self._welcome_message
                else:
                    for i in range(5):
                        client[i].cancel_all_goals()
                    rospy.loginfo("stop the action")
                    time.sleep(1)
                    print self._welcome_message

            elif key == '\x03':
                break
        
        #self.redirect_screen_output(False)


    def action_feedback_cb(self, feedback):
        index = int(feedback.name[-1])
        self._robot_position[index][0] = feedback.x
        self._robot_position[index][1] = feedback.y
        self._robot_position[0] += 1
        if self._robot_position[0] == 5:   # wait for msgs from all five robots
            self._robot_position[0] = 0    # and log them in order
            for i in range(1,6):
                rospy.loginfo("robot" + str(i) + " (% .4f,% .4f) ", \
                self._robot_position[i][0], self._robot_position[i][1])
            rospy.loginfo("--------------------------------")
            position_x = PointArray()
            position_y = PointArray()
            for i in range(1,6):
                position_x.point.append(self._robot_position[i][0])
                position_y.point.append(self._robot_position[i][1])
            self.plot_x.publish(position_x)
            self.plot_y.publish(position_y)

    
    def action_done_cb(self, success, result):
        if(success==3):   # no idea why this flag has to be 3 when success
            index = int(result.name[-1])
            self._robot_position[index][0] = result.x
            self._robot_position[index][1] = result.y
            self._robot_position[0] += 1
            if self._robot_position[0] == 5:   # wait for msgs from all five robots
                self._robot_position[0] = 0    # and log them in order
                for i in range(1,6):
                    rospy.loginfo("robot" + str(i) + " (% .4f,% .4f) ", \
                    self._robot_position[i][0], self._robot_position[i][1])
                rospy.loginfo("action succeeded")
        else:
            rospy.loginfo("action failed")


if __name__ == "__main__":
    rospy.init_node('teleop')
    try:
        teleop = Teleop()
    except rospy.ROSInterruptException:
        pass

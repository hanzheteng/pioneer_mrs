/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***
 * Author: Kevin Watts
 * Modified by Hanzhe Teng
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35

class TeleopKeyboard
{
  public:
    TeleopKeyboard();
    void keyLoop();
  private:
    ros::NodeHandle nh_;
    double linear_x, linear_y, x_scale_, y_scale_;
    ros::Publisher vel_pub_1;
    ros::Publisher vel_pub_2;
    ros::Publisher vel_pub_3;
    ros::Publisher vel_pub_4;
    ros::Publisher vel_pub_5;
};

TeleopKeyboard::TeleopKeyboard():
  linear_x(0),
  linear_y(0),
  x_scale_(2.0),
  y_scale_(2.0)
{
  nh_.param("scale_linear_x", x_scale_, x_scale_);
  nh_.param("scale_linear_y", y_scale_, y_scale_);
  vel_pub_1 = nh_.advertise<geometry_msgs::Vector3>("/robot1/cmd_vel_hp", 1);
  vel_pub_2 = nh_.advertise<geometry_msgs::Vector3>("/robot2/cmd_vel_hp", 1);
  vel_pub_3 = nh_.advertise<geometry_msgs::Vector3>("/robot3/cmd_vel_hp", 1);
  vel_pub_4 = nh_.advertise<geometry_msgs::Vector3>("/robot4/cmd_vel_hp", 1);
  vel_pub_5 = nh_.advertise<geometry_msgs::Vector3>("/robot5/cmd_vel_hp", 1);
}

int kfd = 0;

struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hp");
  TeleopKeyboard teleop_hp;
  signal(SIGINT,quit);
  teleop_hp.keyLoop();
  return(0);
}

void TeleopKeyboard::keyLoop()
{
  char c;
  bool dirty=false;
  bool ROBOT1=false;
  bool ROBOT2=false;
  bool ROBOT3=false;
  bool ROBOT4=false;
  bool ROBOT5=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press number 1-5 to switch the robot.");
  puts("Press q to stop the program");
  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }
    linear_x=linear_y=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
    	  linear_y = 0.1;
    	  linear_x = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
    	  linear_y = -0.1;
    	  linear_x = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_U:
    	  ROS_DEBUG("UP");
    	  linear_x = 0.1;
    	  linear_y = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
    	  linear_x = -0.1;
    	  linear_y = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
    	  linear_x = 0;
    	  linear_y = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_1:
    	  ROBOT1 = !ROBOT1;
    	  if(ROBOT1 == true)
    	    ROS_INFO("ROBOT1 ON");
    	  else
    	    ROS_INFO("ROBOT1 OFF");
    	  break;
    	case KEYCODE_2:
    	  ROBOT2 = !ROBOT2;
    	  if(ROBOT2 == true)
    	    ROS_INFO("ROBOT2 ON");
    	  else
    	    ROS_INFO("ROBOT2 OFF");
    	  break;
    	case KEYCODE_3:
    	  ROBOT3 = !ROBOT3;
    	  if(ROBOT3 == true)
    	    ROS_INFO("ROBOT3 ON");
    	  else
    	    ROS_INFO("ROBOT3 OFF");
    	  break;
    	case KEYCODE_4:
    	  ROBOT4 = !ROBOT4;
    	  if(ROBOT4 == true)
    	    ROS_INFO("ROBOT4 ON");
    	  else
    	    ROS_INFO("ROBOT4 OFF");
    	  break;
    	case KEYCODE_5:
    	  ROBOT5 = !ROBOT5;
    	  if(ROBOT5 == true)
    	    ROS_INFO("ROBOT5 ON");
    	  else
    	    ROS_INFO("ROBOT5 OFF");
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        ROS_INFO_STREAM("You quit the teleop successfully");
        return;
        break;
  	}
		ROS_INFO("Welcome");
    geometry_msgs::Vector3 vel;
    vel.x = x_scale_*linear_x;
    vel.y = y_scale_*linear_y;
    vel.z = 0;
    if(dirty == true)
  	{
  	  if(ROBOT1 == true)
  	    vel_pub_1.publish(vel);
  	  if(ROBOT2 == true)
  	    vel_pub_2.publish(vel);
  	  if(ROBOT3 == true)
  	    vel_pub_3.publish(vel);
  	  if(ROBOT4 == true)
  	    vel_pub_4.publish(vel);
  	  if(ROBOT5 == true)
  	    vel_pub_5.publish(vel);
  	  dirty=false;
  	}
  }
  return;
}

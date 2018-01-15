# pioneer_mrs
## Introduction
ROS package for Pioneer 3-AT Multi-Robot Systems.

<table>
  <tr>
    <th> ROS package </th>
    <th> Description (<I> Capabilities </I>) </th>
    <th> Platform </th>
    <th> Maintainer </th>
  </tr>
  <tr>
    <td> <a href="https://github.com/hanzheteng/pioneer_mrs">
      pioneer_mrs </a> </td>
    <td> <ul>
      <li> Control the (x,y) velocity of the hand point of the robot (a linear system, see [demo](https://photos.app.goo.gl/RM9N5i3zI2EHCJNo1)) </li>
      <li> Friendly interface to control part of or all the robots at the same time </li>
      <li> Task 1: multi-robot trajectory tracking (circular shape, [demo](https://photos.app.goo.gl/eyeclA9uSp1WvkLr2)) </li>
      <li> Task 2: distributed adaptive gradient optimization algorithm </li> </ul> </td>
    <td> <ul>
      <li> <a href="http://robots.ros.org/pioneer-3-at/"> Pioneer 3-AT </a> </li>
      <li> Ubuntu 16.04.3 LTS </li>
      <li> <a href="http://www.ros.org/about-ros/"> Robot Operating System </a> (kinetic) </li>
      <li> <a href="http://wiki.ros.org/ROSARIA"> ROSARIA </a>  package (git-commit-aa8d5f7) </li>
      <li> <a href="http://wiki.ros.org/ROSARIA"> ARIA </a> lib (2.9.1a-ubuntu16-i386) </li> </ul> </td>
    <td> <a href="https://github.com/hanzheteng"> @hanzheteng </a> </td>
  </tr>
</table>

## Usage
Before using this package to drive your robots, you may configure your platform following the below steps:
- All machines (5 robots + 1 laptop) are installed Ubuntu 16.04 and ROS Kinetic
- All robots (onboard computer, actually) are installed and *catkin_make*-ed ARIA lib and ROSARIA package
- All machines need a static IP address and are able to connect to your local WiFi automatically
- Write the IP and hostname information into */etc/hosts* file on all the machines
- Your commander (laptop) is able to SSH to all robots via public key (instead of password)
- Onboard computers are able to get access to the microcontroller via serial post (add user to *dialout* group)

Moreover, there are two suggestions which is not necessary for this system but may be useful for your development.
- You may install a [teamviewer](https://www.teamviewer.us/) software for your laptop and robots so that you can remote login into your onboard computers.
- For IDE, I recommend [RoboWare Studio](http://www.roboware.me/#/home), which is especially designed for ROS developers. Another one I recommend is [KDevelop](https://www.kdevelop.org/), which is a cross-platform IDE.

Then, you can clone this repository by `git clone https://github.com/hanzheteng/pioneer_mrs.git` and `catkin_make` this package in your workspace.

Notice:
1. You must `catkin_make` the ROSARIA package first before you clone and make this package, otherwise it may report errors.
2. Sometimes you need to `catkin_make pioneer_mrs_generate_messages` first before `catkin_make` this package, because the compiler may not be able to find the header file of our messages.

Sample usage (on your laptop):
1. launch one robot (e.g. robot#) </br>
`roslaunch pioneer_mrs single-robot.launch machine:=robot1`
2. launch all five robots </br>
`roslaunch pioneer_mrs multi-robot.launch`
3. launch commander nodes </br>
`roslaunch pioneer_mrs commander.launch`
4. you may choose localization approach like this </br>
`roslaunch pioneer_mrs single/multi-robot.launch pose:=odom/vicon`

## Robot Operating System Info
### ROS Node Info
<table>
  <tr>
    <th> Node name </th>
    <th> Sub topic </th>
    <th> Pub topic </th>
    <th> Description </th>
    <th> Maintainer </th>
  </tr>
  <tr>
    <td> /vicon </td>
    <td> --- </td>
    <td> /vicon/robot#/robot# </td>
    <td> publish translation and rotation info </td>
    <td> <a href="http://wiki.ros.org/vicon_bridge"> vicon_bridge </a> package </td>
  </tr>
  <tr>
    <td> /robot#/RosAria </td>
    <td> /robot#/RosAria/cmd_vel </td>
    <td> /robot#/RosAria/pose
      <br> /robot#/RosAria/... </td>
    <td> receive velocity commands and execute </td>
    <td> <a href="http://wiki.ros.org/ROSARIA"> ROSARIA </a> package </td>
  </tr>
  <tr>
    <td> /robot#/handpoint_node </td>
    <td> /robot#/cmd_vel_hp
      <br> /robot#/RosAria/pose
      <br> /vicon/robot#/robot# </td>
    <td> /robot#/RosAria/cmd_vel </td>
    <td> translate hand-point commands </td>
    <td> @hanzheteng </td>
  </tr>
  <tr>
    <td> /robot#/trajectory_node </td>
    <td> /robot#/mission_state
      <br> /robot#/trajectory
      <br> /robot#/RosAria/pose
      <br> /vicon/robot#/robot# </td>
    <td> /robot#/cmd_vel_hp </td>
    <td> receive trajectory commands and execute </td>
    <td> @hanzheteng </td>
  </tr>
  <tr>
    <td> /robot#/algorithm_node </td>
    <td> /robot#/mission_state
      <br> /robot#/comm_state
      <br> /robot#/RosAria/pose
      <br> /vicon/robot#/robot# </td>
    <td> /robot#/cmd_vel_hp </td>
    <td> execute designed algorithm </td>
    <td> @hanzheteng </td>
  </tr>
  <tr>
    <td> /commander/teleop </td>
    <td> --- </td>
    <td> /robot#/mission_state
      <br> /robot#/cmd_vel_hp </td>
    <td> commander & teleoperation node </td>
    <td> @hanzheteng </td>
  </tr>
  <tr>
    <td> /commander/trajectory_cmd </td>
    <td> --- </td>
    <td> /robot#/trajectory </td>
    <td> send designed trajectory info </td>
    <td> @hanzheteng </td>
  </tr>
  <tr>
    <td> /commander/comm_state_cmd </td>
    <td> --- </td>
    <td> /robot#/comm_state </td>
    <td> create a communication graph between robots </td>
    <td> @hanzheteng </td>
  </tr>
</table>

### ROS Topic Format
<table>
  <tr>
    <th> Topic name </th>
    <th> Format </th>
    <th> Message file </th>
    <th> Description </th>
  </tr>
  <tr>
    <td> vicon/robot#/robot# </td>
    <td> see wiki </td>
    <td> [TransformStamped.msg](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html) </td>
    <td> --- </td>
  </tr>
  <tr>
    <td> /robot#/RosAria/pose </td>
    <td> see wiki </td>
    <td> [Odometry.msg](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) </td>
    <td> --- </td>
  </tr>
  <tr>
    <td> /robot#/RosAria/cmd_vel </td>
    <td> see wiki </td>
    <td> [Twist.msg](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) </td>
    <td> --- </td>
  </tr>
  <tr>
    <td> /robot#/cmd_vel_hp </td>
    <td> float64 x
      <br> float64 y
      <br> float64 z </td>
    <td> [Vector3.msg](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) </td>
    <td> only use 2D velocity (z=0) </td>
  </tr>
  <tr>
    <td> /robot#/trajectory </td>
    <td> float64[] positions
      <br> float64[] velocities
      <br> float64[] accelerations
      <br> float64[] effort
      <br> duration time_from_start </td>
    <td> [JointTrajectoryPoint.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) </td>
    <td> --- </td>
  </tr>
  <tr>
    <td> /robot#/mission_state  </td>
    <td> bool trajectory
      <br> bool algorithm </td>
    <td> MissionState.msg </td>
    <td> set mission start or stop </td>
  </tr>
  <tr>
    <td> /robot#/comm_state </td>
    <td> bool[5] state </td>
    <td> CommunicationState.msg </td>
    <td> one row of the communication graph </td>
  </tr>
</table>

## Update Log
- V1.0 (Jan. 4, 2018) Basic multi-robot control framework
- V1.1 (Jan 15, 2018) The MRS control framework works well with new features, but the algorithm node need to be updated

## Debug FAQ
Most of the time, you can use `roswtf` command to diagnose your problem.
1. If `roslaunch` or `rosrun` cannot autocomplete by `Tab` key, first check if your search path covers the package by `env | grep ROS`. If it already exist in the path, try `rospack profile` or reboot your system.
2. If prompt `process has died, exit code -11`, you may access data in the code which was not initialized.
3. If prompt `The following roslaunch remote processes failed to register: * xxxx (timeout 10.0s)`, your network may have a problem, including hosts file, ROS_MASTER_URI, env loader, etc. (e.g. the hostname of your laptop was not exactly the same as you set in /etc/hosts file)

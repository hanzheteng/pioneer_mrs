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
      <li> Control the hand point of a robot (as a linear system) via feedback linearization </li>
      <li> Distributed launch for 1-5 robots from your own laptop as a master </li>
      <li> Keyboard teleoperation for 1-5 robots at the same time </li>
      <li> Each robot can track a desired trajectory </li> </ul> </td>
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
- all machines (5 robots + 1 laptop) are installed Ubuntu 16.04 and ROS Kinetic
- all robots (onboard computer, actually) are installed and *catkin_make*-ed ARIA lib and ROSARIA package
- all machines need a static IP address and are able to connect to your local WiFi automatically
- write the IP and hostname info of all machines into */etc/hosts* file on all the machines
- your commander (laptop) is able to SSH to all robots via public key (instead of password)
- onboard computers are able to get access to the microcontroller via serial post (add user to *dialout* group)

Moreover, there are two suggestions which is not necessary for this system but may be useful for your development.
- you may install a *teamviewer* software for your laptop and robots so that you can remote login into your onboard computers.
- For IDE, I recommend [RoboWare Studio](http://www.roboware.me/#/home), which is especially designed for ROS developers. The second chioce is [KDevelop](https://www.kdevelop.org/), which is a cross-platform IDE.

Then, you can clone this repository by
`git clone https://github.com/hanzheteng/pioneer_mrs.git`
and `catkin_make` this package in your workspace. Notice: you must `catkin_make` the ROSARIA package first before you clone and make this package, otherwise it may report errors.

Sample usage (on your laptop):
1. launch one robot (e.g. robot1) </br>
`roslaunch pioneer_mrs single-robot.launch machine:=robot1`
2. launch all five robots </br>
`roslaunch pioneer_mrs multi-robot.launch`
3. run commander node </br>
`roslaunch pioneer_mrs commander.launch`
4. send trajectory info to one robot (e.g. robot1) </br>
`roslaunch pioneer_mrs trajectory.launch machine:=robot1`

## Update Log
- V1.0 (Jan. 4, 2018) Basic multi-robot control framework

## Debug FAQ
Most of the time, you can use `roswtf` command to diagnose your problem.
1. If `roslaunch` or `rosrun` cannot autocomplete by `Tab` key, first check if your search path covers the package by `env | grep ROS`. If it already exist in the path, try `rospack profile` or reboot your system.
2. If prompt `process has died, exit code -11`, you may access data in the code which was not initialized.
3. If prompt `The following roslaunch remote processes failed to register: * xxxx (timeout 10.0s)`, your network may have a problem, including hosts file, ROS_MASTER_URI, env loader, etc. (e.g. the hostname of your laptop was not exactly the same as you set in /etc/hosts file)

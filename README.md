ardrone_vicon
====
This repository documents some works on an ardrone project using vicon.


Systems work on
-----
Ubuntu14              ROS indigo  <br>
Ubuntu16                  ROS kinetic

How to use vicon to communicate with your ROS
-----
We need a router to help the computer connect with vicon: close DHCP function and insert both vicon and your computer into the ethernet interface.
<br> Then we need to put vicon and your computer under one network segment：the default IP address of vicon is 192.168.10.1, so we can configure the router's IP address(now we uses it as a switch) as 192.168.10.254 and your computer's IP address as 192.168.10.253.
<br> Now, all the hardware connection is finished.
<br>

How to use this repository
---
This repository consists of three packages, `ardrone_autonomy` is the driver of ardrone which is the basis of all ardrone project; `vicon_bridge` is created to connect ROS with vicon and receive data. These two packages have been created by some genius long before.
<br> `ardrone_vicon` is a package that performs several functions based on these two packages. You can get started quikly using the following command:
First, we need to create a ROS workspace:
```
mkdir vicon
cd vicon
mkdir src
cd src
catkin_workspace_init
cd ..
catkin_make
```
Then, you can download packages:
```
cd src
git clone https://github.com/Shicheng-Liu/ardrone_vicon.git
cd ..
catkin_make
```
Finished!

ardrone_vicon
====
This repository documents some works on an ardrone project using vicon.


Systems work on
-----
Ubuntu14              ROS indigo  <br>
Ubuntu16                  ROS kinetic

How to use vicon to communicate with your ROS
-----
For now, you have _vicon_ and two computers: one is ubuntu computer and the other one is windows computer where the _vicon tracker_ is installed.<br>
We need a router to help the computers connect with vicon: close DHCP function and insert both vicon and your computers into the ethernet interfaces.
<br> Then we need to put vicon and your computers under one network segment：the default IP address of vicon is 192.168.10.1, so we can configure the router's IP address(now we uses it as a switch) as 192.168.10.254 and your ubuntu computer's IP address as 192.168.10.253. Your windows computer is 192.168.10.1 since it has _vicon tracker_ already been installed before.
<br> Now, all the hardware connection is finished.
<br>

Download and set up workspace
---
This repository consists of three packages, `ardrone_autonomy` is the driver of ardrone which is the basis of all ardrone projects; `vicon_bridge` is created to connect ROS with vicon and receive data. These two packages have been created by some genius long before.
<br> `ardrone_vicon` is a package that performs several functions based on these two packages. You can get started quikly using the following command:
<br>First, we need to create a ROS workspace:
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
<br>You may also want to add path to the evironment:
```
gedit ~/.bashrc

///add this line at the bottom
source ~/vicon/devel/setup.bash
```
Make your change into effect:
```
source ~/.bashrc
```

How to use functions
--
Before using _ardrone_vicon_, make sure you have followed all the steps above successfully.
<br> Open _vicon tracker_ and choose object, connect the WIFI of your ardrone.
<br> Open a terminal:
```
roslaunch ardrone_vicon ardrone.launch
```
Open another terminal:
```
rosrun ardrone_vicon ardrone_vicon_node
```
Since we have added path to the environment, we have no need to source the environment now. Now you will see a menu like this:
        
   ![menu](https://github.com/Shicheng-Liu/ardrone_vicon/blob/master/menu.png)
<br>
Choose y to take off is always the first step, and then you can choose functions you want.
<br>

Funtions Introduction
-
**k:keyboard control**
<br> You will say a new menu like this after you press k:

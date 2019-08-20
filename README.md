ardrone_vicon
====
This repository documents some works on an ardrone project using vicon.


Systems work on
-----
Ubuntu14         &nbsp;    &nbsp;   ROS indigo  <br>
Ubuntu16           &nbsp;    &nbsp;          ROS kinetic

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
**k  :  keyboard control**
<br> You will see a new menu like this after you press k:

![image](https://github.com/Shicheng-Liu/ardrone_vicon/blob/master/keyboard%20control.png)
<br>
The drone will act exactly like the menu says.
<br>
<br>
**a  :  calibration**
<br> 
The drone will calibrate its angle and position, the angle will be less than 8 degree and the position will be about the origin.
**NOTE** : In my scenario, the coordinate system of drone is different from that of _vicon_ : The x axis of drone is actually the -y axis of _vicon_ and the y axis of drone is actually the -x axis of _vicon_. Accordingly, if your situation is different, please go to _ardrone_vicon.cpp_ and change the coordinate system.
<br>
<br>
**b  :  trajectory**
<br>
The drone will move to (0,0) (1,1) (1,-1) (-1,-1) (-1,1) (0,0)
<br>
<br>
**c  :  pose measurement**
<br>
You will see the basic state if your drone, like the position in 3D dimension and the angle of the drone.
<br>
<br>
**f  :  target follow**
<br> 
First, you need to add a new object in _vicon tracker_ and name it _target_.<br>
Then you can use this funtion: the drone will track target wherever you move the target. This funtion uses simple difference to realize following and the drone will be about 2 meters away from the target.
<br>
<br>
**p  :  potential field track**
<br>
This function is similiar to _target follow_ but uses a different method [potential field](http://kovan.ceng.metu.edu.tr/~asil/old/_1./wh2.html)
<br> The drone will track your target.
<br>
<br>
**l  :  potential field repulse**
<br>
This funtion is at the opposite of track: the drone will move away from the object as if the target has a repulsive power.
<br> First, you need to add sa new object in _vicon_tracker_ and name it _obstacle_.
<br> Then you can find the drone moving away from the obstacle.
<br>
<br>
**m  :  potential field track with obstacle**
<br>
This function is a combination of the last two functions, you can track a tracking while avoiding an obstacle.

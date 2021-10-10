# TurtleBot 3 Starter Code for Mobile Robotics EECE 5550 - Fall 2021

This repo will hold the starter codes for the lab assignemnts. You will build solutions on top of this repository. This tutorial assumes that you have successfuly installed Ubuntu 20.04 and ROS Noetic on your machine. 

## Simulation Setup
0. Head to the TurtleBot 3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  
1. First read the Overview and Features sections in the manual to get yourself familiar with the platform. 

2. Go to the Section 3.1 PC Setup. Skip part 3.1.1 and 3.1.2 since we already installed Ubuntu and ROS and continue with the part 3.1.3 which installs the dependencies for the TurtleBot 3 packages. Make sure that you selected the Noetic version from the top: 

3. You might get the following error: `no matches found: ros-noetic-rqt*` If you see this error, replace the line `ros-noetic-rqt*` with `ros-noetic-rqt` and run the installation command again. 

4. After you finish this section, the control software for the actual hardware is installed. Now, we will install the simulation. To do so create a ROS workspace as we showed in the class. In case you need a refresher, check out the ROS tutorials: http://wiki.ros.org/catkin/Tutorials/create_a_workspace  

5. Follow the steps below to install the TurtleBot 3 simulator 
    * `cd ~/catkin_ws/src` 
    * `git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git `
    * `cd ~/catkin_ws `
    * `catkin build`
    * `source ~/catkin_ws/devel/setup.bash `
6. To confirm the TurtleBot 3 simulation, run the following commands: 
    * `export TURTLEBOT3_MODEL=burger` 
    * `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch `

7. You should a Gazebo screen with TurtleBot 3 loaded: 
![TurtleBot 3](images/gazebo_empty.png)
 
8. If you see this section, the TurtleBot 3 simulation is successfully installed. If you don’t get to this point, check previous steps carefully and make sure you didn’t miss any commands. 

# iFollow-technical-test
ROS framework applications Test:

## Table of contents: 
*  Setting up the test environment / Mise en place de l’environnement de test
*  Control multiplexer / Multiplexeur de commande
*  Remote teleoperation / Téléopération à distance
*  Sending of Goal determined by a visual tag / Envoi de Goal déterminé par un tag visuel
* Using an image capture / Utilisation d’une capture d’image 

## 1. Setting up the test environment / Mise en place de l’environnement de test: 
* Creating the package under the workspace catkin_ws and build the workspace
```
$ catkin_create_pkg test rospy roscpp std_msgs sensor_msgs
$ catkin_make
```

* Install turtlebot3 packages:
```
$ git clone -b noetic-devel  https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

* Choosing the turtlebot3 model burger
```
$ gedit ~/.bashrc
```
add this to the bottom of the file: export TURTLEBOT3_MODEL=burger

* Robot simulation on Gazebo:I used turtlebot3_world but you cn use any other map of your choice from turtlebot3_simulations
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

![gazebo](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/1e92bdeb-7cad-4a59-b6aa-92d1baa0d9d1)

* Robot simulation on Rviz:this command transfers the gazebo simulations to rviz in real time
```
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

![rviz](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/b88d9769-212c-4115-8785-1b9431cf55fc)

* Teleoperate turtlebot3 using the keyboard:
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

![teleop_key](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/6e8d000a-bd72-4e09-a940-be9f177fe1cc)

* Result after modifying linear and angular velocity: 

![rviz_slam](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/1f75ce52-75df-4862-9e15-6f056b017b82)


* Localization and Navigation:
  



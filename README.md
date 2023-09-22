# iFollow-technical-test
ROS framework applications Test:

## Table of contents & duration it took to be completed
*  Setting up the test environment / Mise en place de l’environnement de test (4hours)
*  Control multiplexer / Multiplexeur de commande (2hours)
*  Remote teleoperation / Téléopération à distance (3hours)
*  Sending of Goal determined by a visual tag / Envoi de Goal déterminé par un tag visuel (4 hours)
*  Using an image capture / Utilisation d’une capture d’image (0.5 hours)

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

  running the turtlebot3_world and SLAM simulations:
  NB: make sure you have gmapping, amcl, map_saver and move_base installed before
  ```
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
   running teleop_key command in a separate terminal and navigating to map the robot's environment
  
   saving the mapping done through the navigation of the robot:
  ```
  $ rosrun map_server map_saver -f ~/map
  ```

  Result:
  
![map](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/e652c767-3232-437a-8c5a-467bfbdf0c7a)


  launching turtlebot3_world once again, and launching rviz using the saved map: 
  NB: Install dwa-local-planner if you don't have it 
  ```
  $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
  ```
  Result after fixing 2d pose estimate (on the left) then after narrowing down the position estimation represented by the small green arrows (on the right) using      the teleop command: 
  
  ![green](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/344679a8-411a-4409-9a43-49e1a8f1a8a0)

  Finally, set the desired 2d av goal using the 2d pose estimate in RVIZ:

  ![2dnav](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/6a4a7360-eb1c-40a8-ad10-5d0984bc544a)

## 2. Control multiplexer / Multiplexeur de commande:

* First, make Multiplex.cpp executable by adding these two lines to the CmakeLists.txt:

add_executable(Multiplex.cpp /home/ilef/catkin_ws/src/test/src/Mutiplex.cpp)

target_link_libraries(Multiplex.cpp ${catkin_LIBRARIES})

* run the node Multiplex.cpp
* publish the switch value to the switch topic from terminal:
```
$  rostopic pub -1 /switch std_msgs/Int8 '1'
```

publish the velocity command to the according topic from terminal: 
```
$ rostopic pub -1 /cmd_local geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
result: 

![test resut](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/b9fde005-da8a-4a29-b2b1-965cba868857)


if the switcher value is out of the range of considered values: 

![test 2](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/72c02a89-a36b-4a2e-91c9-c255ca823764)



if the switcher value does not correspond to the cmd topic nothing happens

## 3. Remote teleoperation / Téléopération à distance

* Install the MQTT and keyboard input libraries:
```
$ pip install paho-mqtt
$ pip install pynput
```

* run the Teleop_MQTT.py by accessing to its directory and running:
```
 $  python3 Teleop_MQTT.py
```

 * change the velocity using these keys:
  {UP, DOWN} : to increase/decrease linear velocity
  {LEFT, RIGHT} : to increase/decrease angular velocity
  {ESC} : to put back both velocities to 0.0
  
the velocity is published every time it is changed using keypresses to "velocity_topic"

![teleop](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/1815cefa-8967-4766-85a4-74b1dfd12069)

rosrun the MQTT_listener.py: 
```
$ rosrun test MQTT_listener.py
```

![mqtt listener](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/b87e9ca3-7b08-4662-b560-7fa3e6a8b46b)



MQTT_listener publishes to /cmd_web:

![cmd_web_echo](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/4b0fbcc9-b71c-4b84-8187-49034b981f05)


## 4. Sending of Goal determined by a visual tag / Envoi de Goal déterminé par un tag visuel:

* Install opencv and apriltag
* change the "path" variable to the path containing the apriltag
* launch turtlebot3 on gazebo and rviz using commands:
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
* Use teleop_key comand to narrow down the green arrows
rosrun AR_decode.py:

![AR_decode execution](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/349cb3b8-62ed-4d66-af1e-4fa57741722f)

* AR tag recognition:

  ![savedARtag2](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/3c6a327f-4a90-4447-9a5d-11945225952b)

result on rviz:


![navgoal_from_tag](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/4ac4d13b-2c6f-4119-92f1-37f4820eb763)

## 6. Using an image capture / Utilisation d’une capture d’image :

* run Screenshot_recognition.py
* captured image:

![AR_Tag](https://github.com/Ilef27/iFollow-technical-test/assets/74418956/3fcff527-73be-4d97-b4a5-deb1ea6aea1b)

the rest processing is similar to #4

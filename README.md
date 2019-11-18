# Walker Robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This project uses simulation of turtlebot in the customized gazebo environment to navigate by avoiding obstacles in its path.This robot shows the behavior of roomba walker robot.This project uses ROS,Catkin,Gazebo libraries as key libraries.Robot gets the information of obstacle ahead in its path from its sensors.Here, laserscan is used to get the sensor data.

# Dependencies 
1) ROS distro: 'Kinetic'. 
2) Catkin installed
3) Turtlebot packages 
4) Gazebo - latest version, minimum of 7.4

If turtlebot is not installled, use the following commands for installation:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

# Building package 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/vamshibogoju/walker.git
cd ..
catkin_make
```

# Run Gazebo simulation
 
Type the following command in a new terminal:
```
roslaunch walker walker.launch
```
Here rosbag record in not being run.

# Recording using rosbag files 
To record the ros topics use the following commands: 

```
roslaunch walker walker.launch bagrecord:=1
```
The recorder bag file will be saved in the results folder in the package

To record for a specific time, say 20 seconds 
```
roslaunch walker walker.launch bagrecord:=1 secs:=20
```


# Play bag files 
navigate to the folder where bag files are saved,Currently they are saved in the results folder:
 
```
cd ~/catkin_ws/src/walker/results 
```
Use following command to play the saved rosbag file

```
rosbag play record.bag
```
Published topic can be verfied by using following command:

```
rostopic echo /mobile_base/commands/velocity
```
## Cpplint check
Execute the following commands in a new terminal to run cpplint
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp )
```

## Cppcheck 
Execute the following commands in a new terminal to run cppcheck
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp )

```


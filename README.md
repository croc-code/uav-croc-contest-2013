croc-uav-contest-2013
=====================

##Overview

Description of software solution of Croc Team UAV for "Croc UAV Contest 2013".

Solution is build on [ROS Groovy](http://www.ros.org) and uses both native either custom ros nodes. Solution runs under Ubuntu 12.04.

Software is provided "as is" and any warranties are disclaimed! 


##Prerequisites:

1. ROS Groovy. Read about installation [here](http://wiki.ros.org/groovy/Installation). Package to install: `ros-groovy-desktop-full`. Also it's highly recommended to complete [tutorial](http://wiki.ros.org/ROS/Tutorials) of Beginner Level at least. While completing tutorial please take in mind that we use rosbuild package organizing system and not catkin.

2. [OpenCV 2.4.3](http://opencv.org/downloads.html). Detailed info about installation on linux you can find [here](http://docs.opencv.org/trunk/doc/tutorials/introduction/linux_install/linux_install.html).

3. Qt4 (needed for ground station only, though it's nice IDE for ROS programming and building).


Native ROS packages you need (check if they are installed with ROS distributive):

1. [laser_drivers](http://wiki.ros.org/laser_drivers?distro=groovy) - for Hokuyo laser range finders (including 30LX) support.

2. [navigation](http://wiki.ros.org/navigation) - for navigation level of solution (global path planning, costmap building).




##Building solution

1. Create ROS workspace (for example in folder `/home/rosuser/ros/`). Don't forget to add this path to `ROS_PACKAGES_PATH` variable, for example by adding following lines to .bashrc:

```shell
		source /opt/ros/groovy/setup.bash
		ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/rosuser/ros/
```

2. Get solution source code to ROS workspace.

3. Build 3rd-party packages which are placed in `/ros_3rd_party` floder. In folder `/ros_3rd_party` run following commands:

```shell
		rosdep install *
		rosmake *
```

Following packages should be installed after that:

 - control-toolbox - this package contains modules that are useful across all controllers. You can also get sources from [here](http://wiki.ros.org/control_toolbox).

 - px-ros-pkg (optional) - this package provides a ROS interface to the PX4Flow optical flow board. Though the final solution doesn't use optical flow for speed calculations but it uses ultrasonic ranger data from this board. If you are about to use standalone ultrasonic ranger, then you do not need this package and may be you will need rosserial package. You can also get sources from [here](http://wiki.ros.org/px4flow_node).

 - rosserial (optional) - this package is for sending ROS messages over serial links. If you use standalone ultrasonic ranger for altitude calculation may be you will need this one. You can also get sources from [here](http://wiki.ros.org/rosserial).

 - scan-tools - this stack contains tools for manipulating sensor_msgs/LaserScan and sensor_msgs/PointCloud messages. And also it contains Andrea Censi's Canonical Scan Matcher realization. You can also get sources from [here](http://wiki.ros.org/scan_tools).


4. Build packages from folder `/ros_croc`. They all were made by Croc Team except `cyphy_serial_driver` package, which was made by CYPHY Lab Team (Thank you, guys!! You helped us a lot!) and someway fixed and changed by Croc Team. In folder `/ros_croc` run following commands:

```shell
		rosdep install *
		rosmake *
```

Following packages should be installed after that:

- cyphy_serial_driver - low-level logic for communication with mikrokopter flight controller.

- croc_Pose3D - current robot postion aggregation node. Position data is taken from lidar (via scan-matcher node), ultrasonic ranger (via px-ros-pkg or rosserial) and IMU (via cyphy_serial_driver, but not needed really).

- croc_navi - sum of local planner and navigation state machine. It knows about costmap and global path from navigation ros stack and chooses local goal according global path. Also it changes global goals according current navigation state (searching for landmark, returning home, etc).

- croc_command - main control node which performes altitude and horizontal moving\stabilization tasks. It contains logic full of PID controllers, landing and take-off detections and control parameters of different kind. This node gets current position from Pose3D and local goal point from croc_navi and tries to move robot to designated goal.

- croc_detecting_mark_3 - node with OpenCV implementation of landing mark detection. When landmark is found, node publishes coordinates of landing mark center on image.

- croc_CalcLandingPos - node that transforms coordinates on image to physical coordinates on the ground. This physical coordinates are sets as global goal later in croc_navi.

- croc_log - node writes csv-log with data from mostly all solution topics.



##Running solution


1. Go to /launch folder and set parameters.

- robot.launch - solution launch file. Parameters are described in comments in launch file. Do not forget to set device names for Hokuyo, MKFlightController and px4flow\sonar nodes.

- *.yaml - parameters of navigation stack. More detailed information see on [this page](http://wiki.ros.org/navigation).

- map05_40x10_large_filled.png - initial map of testing site for navigation map server. If you dont want it - fill it white, or remove from navigation parameters.

It will be nice if you read and understand the code and every parameter meanings before robotm launching. Remember, this software is provided "as is" and any warranties are disclaimed! 


2. Execute command 

```shell
roslaunch /home/rosuser/ros/launch/robot.launch
```

3. For robot taking off you can execute command:

```shell
rostopic pub -1 /StateCmd croc_command/State '{header: auto, state: 1}'
```shell

Robot should take off. After that you can move robot by command:

```shell
rostopic pub -1 /Goal geometry_msgs/Pose2D '{x: 2.0, y: 0.0, theta: 0.0}'
```

For example this command will move robot in two meters forward (or in some other direction depending on your hokuyo installation). Attention! If navigation_on parameter in croc_navi is set to true, robot will try to perform contest task instead of going to designated goal! Be careful.



##Ground Station

Ground Station is a small software, based on qt4 & ROS rviz package for control robot and getting some telemetry from it (position, speed, video, etc). Also it has some wind control - for gazebo emulation purposes only. It's created as ros node, so you need ROS Groovy and Qt4 on workstation where it will be run. Source code is placed in folder /ros_gs, node has name croc_gs.

To build croc_gs in folder /ros_gs run following commands:

```shell
rosdep install *
rosmake *
```

To run this node complete following commands:

```shell
ROS_MASTER_URI=http://192.168.0.99:11311 
rosrun croc_gs croc_gs
```

where http://192.168.0.99:11311 is the URL of master roscore running on robot. Master roscore must be running during the start. Ground Station GUI should appear after that. Press btton Start to launch robot.








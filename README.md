# ENPM 667 Course Project-01 - Paper Implementation 
## Reference Paper 
> - **Title:** _A Novel Potential Field Controller for Use on Aerial Robots_
> - **Authors:** _Alexander C. Woods_ and _Hung M. La_, _Senior Member, IEEE_

## To run the ROS + GAZEBO simulation 

- **Package Name:** _sjtu-drone_
- **Launch file:** _launch/ardrone_waypoint.launch_
- **Script file:** _src/potential_field_control.py_


Add the given package folder (sjtu-drone) in your catkin_ws/src.

1. Open a terminal and navigate to `cd catkin_ws` and run `catkin_make`.
2. Run `roslaunch sjtu_drone ardrone_waypoint.launch` to launch the drone in the custom world. If it doesn't work, you may run `roslaunch sjtu_drone simple.launch`.
3. In another terminal, run:  `rosrun sjtu_drone potential_field_control.py`

NOTE: Uncomment the corresponding lines (17-34) in the python script depending on which Figure of the original paper you wish to plot.
For example: For Fig 14, uncomment lines 25,26,98,140,160 and comment lines 90,91,134,168,173,174. Roslaunch the launch file and run the conroller python script as mentioned before. 



## To run the MATLAB/SIMULINK simulation

- **Folder Name:** _ARDrone_extended_PFC_
- **Simulink File:** _potential_field_controller.slx_
- **Script to set Waypoints:** _getWaypoints.m_
- **Script to set Obstacles:** _getObstaclepoints.m_
- **Script to load variables & start simulation:** _setupPFC.m_


To run the Simulink program and model,

1. Open MATLAB and change the present directory to inside _ARDrone_extended_PFC_.
2. Open `getWaypoints.m` and add the details about the desired target points.
3. Open `getObstaclepoints.m` and add the X-Y coordinate details of the obstacle(s).
4. Run `setupPFC.m`, this should open the `potential_field_controller.slx` SIMULINK file.
5. Click the green run button on the top to run the simulation, this should open up an X-Y graph of the drone's positions.


<!-- # About sjtu_drone # -->

<!-- Gazebo-9 and Ubuntu 18.04 compatible SJTU Drone package

sjtu_drone is a quadrotor simulation program forked from ['tum_simulator'] (http://wiki.ros.org/tum_simulator) , which is developed with ROS + Gazebo. It is used for testing visual SLAM algorithms aiding with different sensors, such as IMU, sonar range finder and laser range finder. Here by 'sjtu', it means Shanghai Jiao Tong University. Currently, this program is used for testing algorithms for [UAV contest in SJTU](http://mediasoc.sjtu.edu.cn/wordpress)

# Requirements #
This package is compatible with ROS Melodic version (Ubuntu 18.04). Existing versions on the internet support at most until Gazebo 7. After Gazebo 8.0, the API has gone significant changes; therefore, it was necessary to adapt the package to Gazebo 8.0+ API. As the default version of Gazebo coming with ROS Melodic is 7.0, it is suggested that do not use the full installation but the desktop installation.
```
$ sudo apt-get install ros-melodic-desktop
```
# Download and Compiling #
```
$ cd <catkin_ws>/src
$ git clone https://github.com/tahsinkose/sjtu-drone.git
$ cd <catkin_ws>
$ catkin build
```

Here <catkin_ws> is the path of the catkin workspace. Please refer to the [tutorial](http://wiki.ros.org/ROS/Tutorials) about how to create a catkin workspace in ROS.

# Run
The simplest way is calling after you have built the workspace successfully.

```
$ cd <where you check out the code>
$ source devel/setup.bash
$ roslaunch sjtu_drone simple.launch
```
# Running with keyboard
In second terminal:

```
$ rosrun sjtu_drone drone_keyboard
```


# Read sensor data from ROS topics #
One can use [rqt_gui](http://wiki.ros.org/rqt_gui) to have an extensive amount of utilities for topic visualization and manipulation. Some of the useful topics reside below.
```
forward looking camera :  /drone/front_camera/image_raw
downward looking camera: /drone/down_camera/image_raw
sonar data:  /drone/sonar
laser range data: /drone/laser
``` -->

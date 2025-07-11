# Franka-Emika-Admittance-Control
This repository contains code to perform basic admitance control using a Franka Emika reasearch arm. It is structured like a ROS2 project and package, however ROS2 is not used directly for the control.

ROS2 is used for the transfer of data out of the libFranka control loop to other devices on the network.

![admittance](https://github.com/user-attachments/assets/bbabe633-e7fa-4fd6-9df8-55e839cbd5c3)


## Setup
A lot of assumptions about system configuration are currently hardcoded in this project.
1. Robot is assumed to be at 192.168.18.10 with an alias of panda0.robot
2. Sensor is assumed to be at 192.168.18.12 with an alias of ftsensor
3. netft library for sensor integration depneds on tinyxml, curlpp, and asio which might not be present on your system. These dependencies are only required on the workstation, however being able to build the project on your local system is nice.
## Hardware
A franka emika robot arm with workstation is the robot used while an Axia80-M8 Force/Torque sensor is used to detect load at the robots wrist.

https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M8

## Workflow
1. Run ```./move.sh``` to copy project files to project directory on workstation. Mush have SSH key added to workstation otherwise you might be prompted.
2. On workstation, run ```colcon build``` to build the project in the space it will be run.

## Running

Once the project is built, source it with

 ```source install/setup.bash```

Run the executables with

 ```ros2 run data_bridge <executable>```

Available executables are as follows and all require the robot IP:

`admittance`: the primary point of the project. Sets up sensor and uses data to perform admittance control while keeping the EE locked to XYZ only. So no roll, pitch, or yaw. Additionally takes an option for what mode to run: TRACK is the defualt behavior while SPRING and SPRINGDEMO can be used for testing.

`white_light`: impedance control example

`cartesian_impedance_control`: impedance control example with pose tracking using spring-damper controls.

## ROS2 Integration

The admittance executable currently publishes a large selection of data over the `/robot_data` topic to allow other systems to perform analysis. The data_analysis package in this project is a simple plotting package that can be used for this analysis so long as it is run on any computer on the robots network.

To pull up the live plots of the robots state and the control loop torques run:

```ros2 launch data_analysis report.xml```

after building and sourcing the project on a different machine from the robot.

### Topics

`/robot_data:data_interfaces/msg/Robot`

### Packages

`data_bridge`: contains the admittance controller as well as some other examples for controlling the Franka. Publishes to `/robot_data`

`data_analysis`: contains several different nodes that listen to `/robot_data` and plot data live. Launch file runs all of them.

`data_interfaces`: contains `Robot` msg type used by the `/robot_data` topic. This type is a huge grab bag for all of the data we desire from the robot.

## Credit

F/T sensor integration code was stripped down and modified based on Grzegorz Bartyzel's project here:

https://github.com/gbartyzel/ros2_net_ft_driver/tree/rolling

Thread Safe queue implementation is provided by K-Adam:

https://github.com/K-Adam/SafeQueue

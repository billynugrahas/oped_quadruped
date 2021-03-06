# oped
oped - Quadruped ROS Package

OPED is open source development framework for mammal-type quadruped robot using Dynamixel AX-12/AX-18 series. This framework is based on champ controller library [*"𓃡 CHAMP Quadruped Controller ROS Package"*](https://github.com/chvmp/champ).

The software has been modified to control the robot's joints using Dynamixel AX-12/AX-18 series as the actuator. `

Core Features:
- Fully Autonomous (using ROS navigation Stack).
- [Setup-assistant](https://github.com/chvmp/champ_setup_assistant) to configure newly built robots.
- Gazebo simulation environment.
- Lightweight C++ header-only [library](https://github.com/chvmp/libchamp) that can run on both SBC and micro-controllers.

Supported Hardware:

LIDAR:
- RPLidar A1M8 - WIP
- YDLIDAR X4

IMU:
- MPU6500 - WIP
- MPU6050 - WIP

SBC:
- Raspberry Pi 3 model B+

    This should also work on Single Board Computers that support Ubuntu 18 capable of running ROS Navigation Stack and have a I2C peripheral devices capable of running IMU.

ACTUATORS:
- Dynamixel AX12

TESTED ON:
- Ubuntu 18.04 (ROS Melodic)

## 1. Installation

1.1. Clone and install all dependencies:

    sudo apt install -y python-rosdep
    sudo apt install -y ros-melodic-dynamixel-sdk
    cd <your_ws>/src
    git clone https://github.com/billynugrahas/oped_quadruped
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    
    

1.2. [On RaspberryPi] Install i2c dev, imu_tools, and lidar

    sudo apt-get install i2c-tools python-smbus
    sudo apt-get install ros-melodic-imu-tools
    cd <your_ws>/src
    git clone https://github.com/Slamtec/rplidar_ros.git
    git clone https://github.com/YDLIDAR/ydlidar_ros
    
1.3. Create udev rules (for hardware)

    1 udev rules for ydlidar
        roscd ydlidar_ros/startup
        sudo chmod 777 ./*
        sudo sh initenv.sh
     2 udev rules for U2D2 (USB TO DYNAMIXEL 2)
        cd <your_ws>/src/oped/hardware_install
        sudo chmod 777 ./*
        sudo sh u2d2_dynamixel.sh
        
    
1.4. Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws/>/devel/setup.bash
    
 

## 2. Walking demo in Gazebo:
2.1.1. Run the base driver and gazebo:

    roslaunch oped_config gazebo.launch rviz:=true
    
  You can select the world, just edit the gazebo.launch on oped_gazebo package. The world is in world folder on oped_gazebo package.
  
2.1.2. Run the teleop node:

    roslaunch oped_teleop teleop.launch
   Hold the key until the robot move.
   
### 2.3. Autonomous Navigation (using champ's robot model):

2.3.1. Run the Gazebo environment: 

     roslaunch oped_config test_gazebo_stable.launch

2.3.2. Run amcl and move_base:

    roslaunch oped_config test_navigate_stable.launch rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.
--------------------------------
## WIP - need to tune some parameters of amcl, slam, and path planning on oped_config package
--------------------------------
## WIP - autonomous navigation on maze using champ's robot model
    roslaunch oped_config gazebo_stable.launch
    roslaunch oped_config navigate_stable.launch rviz:=true

--------------------------------
## WIP - autonomous navigation on maze using my own robot

    roslaunch oped_config gazebo.launch
    roslaunch oped_config navigate.launch rviz:=true



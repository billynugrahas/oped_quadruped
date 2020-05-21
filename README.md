# anggada
Anggada - Quadruped Controller ROS Package


ANGGADA is open source development framework for quadruped robot using Dynamixel AX-12/AX-18 series . This framework is modified from champ library [*"𓃡 CHAMP Quadruped Controller ROS Package"*](https://github.com/chvmp/champ).

Core Features:
- Fully Autonomous (using ROS navigation Stack).
- [Setup-assistant](https://github.com/chvmp/champ_setup_assistant) to configure newly built robots.
- Gazebo simulation environment.
- Lightweight C++ header-only [library](https://github.com/chvmp/libchamp) that can run on both SBC and micro-controllers.

Supported Hardware:

LIDAR:
- RPLidar A1M8
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
    git clone https://github.com/billynugrahas/anggada
    git clone https://github.com/chvmp/champ_teleop
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    
    

1.2. [On RaspberryPi] Install i2c dev

    sudo apt-get install i2c-tools python-smbus
    
    
1.2. Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws/>/devel/setup.bash
    

cmake_minimum_required(VERSION 2.8.3)
project(oped_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   oped_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


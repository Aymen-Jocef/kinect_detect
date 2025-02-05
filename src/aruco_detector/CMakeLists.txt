cmake_minimum_required(VERSION 3.0.2)
project(aruco_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/aruco_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

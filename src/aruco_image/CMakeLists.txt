cmake_minimum_required(VERSION 3.0.2)
project(aruco_image)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

## Declare catkin package
catkin_package()

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ## Install Python scripts
# catkin_install_python(PROGRAMS
#   scripts/aruco_image.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/image_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 3.5.1)
project(reswarm_dmpc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)

# TODO: Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/GainsWnXi.cfg
#   cfg/GainsPID.cfg
#   cfg/VisualServoing.cfg
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

catkin_python_setup()
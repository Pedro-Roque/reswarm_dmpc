cmake_minimum_required(VERSION 3.5.1)
project(reswarm_dmpc)

set(qpoases ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers/formation/acado/qpoases)
set(formation_src ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers/formation/acado)
set(acado_leader_src ${formation_src}/leader)
set(acado_subleader_src ${formation_src}/sub_leader)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  message_generation
)

add_service_files(
  FILES
  GetControl.srv
  SetWeights.srv
)

# TODO: Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/weightGenerationExample.cfg
# )

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)




# TODO(@Pedro-Roque): Set ARM optimizations for MLP
# if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
#   message("enabling ARM neon optimizations")
# endif()

# Set default include paths and directories
include_directories(
  include
  ${qpoases}/INCLUDE
  ${qpoases}/SRC
  ${catkin_INCLUDE_DIRS}
  ${formation_src}
)

### LEADER MPC SECTION
# 
# Create the MPC solver library
add_library(leader_mpc_lib
  ${qpoases}/SRC/Bounds.cpp
  ${qpoases}/SRC/Constraints.cpp
  ${qpoases}/SRC/CyclingManager.cpp
  ${qpoases}/SRC/Indexlist.cpp
  ${qpoases}/SRC/MessageHandling.cpp
  ${qpoases}/SRC/QProblem.cpp
  ${qpoases}/SRC/QProblemB.cpp
  ${qpoases}/SRC/SubjectTo.cpp
  ${qpoases}/SRC/Utils.cpp
  ${qpoases}/SRC/EXTRAS/SolutionAnalysis.cpp
  ${acado_leader_src}/acado_qpoases_interface.cpp
  ${acado_leader_src}/acado_integrator.c
  ${acado_leader_src}/acado_solver.c
  ${acado_leader_src}/acado_auxiliary_functions.c
)

target_include_directories(leader_mpc_lib PUBLIC 
  ${acado_leader_src}/
  ${qpoases}
  ${qpoases}/INCLUDE
  ${qpoases}/SRC 
)

add_library(leader_ctl ${formation_src}/../leader_ctl.cpp)
target_link_libraries(leader_ctl leader_mpc_lib ${catkin_LIBRARIES})

add_executable(leader_ctl_node nodes/leader_ctl_node.cpp)
target_link_libraries(leader_ctl_node leader_ctl leader_mpc_lib )


# NMPC libraries


# add_library(subleader_mpc_lib
#   ${qpoases}/SRC/Bounds.cpp
#   ${qpoases}/SRC/Constraints.cpp
#   ${qpoases}/SRC/CyclingManager.cpp
#   ${qpoases}/SRC/Indexlist.cpp
#   ${qpoases}/SRC/MessageHandling.cpp
#   ${qpoases}/SRC/QProblem.cpp
#   ${qpoases}/SRC/QProblemB.cpp
#   ${qpoases}/SRC/SubjectTo.cpp
#   ${qpoases}/SRC/Utils.cpp
#   ${qpoases}/SRC/EXTRAS/SolutionAnalysis.cpp
#   ${acado_subleader_src}/acado_qpoases_interface.cpp
#   ${acado_subleader_src}/acado_integrator.c
#   ${acado_subleader_src}/acado_solver.c
#   ${acado_subleader_src}/acado_auxiliary_functions.c
# )

# target_include_directories(subleader_mpc_lib PUBLIC 
#   ${acado_subleader_src}/
#   ${qpoases}
#   ${qpoases}/SRC
#   ${qpoases}/INCLUDE  
# )

# # Add ROS controller interface for Leader and Follower
# add_library(leader_ctl ${formation_src}/../leader_ctl.cpp)
# # add_dependencies(leader_ctl ${leader_ctl_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(leader_ctl leader_mpc_lib ${catkin_LIBRARIES})

# # Add executable nodes
# add_executable(leader_ctl_node nodes/leader_ctl_node.cpp)
# add_dependencies(leader_ctl_node leader_ctl leader_mpc_lib )
# target_link_libraries(leader_ctl_node leader_ctl leader_mpc_lib )


# add_executable(test_acado_leader ${acado_leader_src}/test.c)
# target_link_libraries(test_acado_leader leader_mpc_lib)

# add_executable(test_acado_subleader ${acado_subleader_src}/test.c)
# target_link_libraries(test_acado_subleader subleader_mpc_lib)

# Add ROS Nodes in C++
# add_executable(leader_ctl ${formation_src}/../leader_ctl.c)
# target_link_libraries(leader_ctl leader_mpc_lib ${catkin_LIBRARIES})

# add_executable(subleader_ctl ${formation_src}/../subleader_ctl.c)
# target_link_libraries(subleader_ctl leader_mpc_lib ${catkin_LIBRARIES})
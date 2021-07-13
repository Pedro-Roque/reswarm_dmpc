cmake_minimum_required(VERSION 3.5.1)
project(reswarm_dmpc)

set(qpoases ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers/formation/acado/qpoases)
set(formation_src ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers/formation/acado)

# ISS Tests
set(acado_unit_test_src ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers/setpoint/acado/acado_setpoint)
set(acado_leader_src ${formation_src}/leader)
set(acado_subleader_src ${formation_src}/sub_leader)

# Ground tests
set(acado_unit_test_src_gnd ${acado_unit_test_src}_gnd)
set(acado_leader_src_gnd ${acado_leader_src}_gnd)
set(acado_subleader_src_gnd ${acado_subleader_src}_gnd)

set(controllers_src ${CMAKE_CURRENT_SOURCE_DIR}/src/reswarm_dmpc/controllers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES 
  InformationStamped.msg
  AcadoStatusStamped.msg
  DMPCTestStatusStamped.msg
)

add_service_files(
  FILES
  GetControl.srv
  SetWeights.srv
)

# generate_dynamic_reconfigure_options(
#   cfg/weightGenerationExample.cfg
# )

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)

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
  ${qpoases}/SRC/acado_dummy_file.c
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

# Acado MPC solver interface for leader
add_library(leader_ctl ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(leader_ctl leader_mpc_lib ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(leader_ctl_node nodes/leader_ctl_node.cpp)
target_link_libraries(leader_ctl_node leader_ctl leader_mpc_lib )


### SUBLEADER MPC SECTION
# 
# Create the MPC solver library
add_library(subleader_mpc_lib
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
  ${qpoases}/SRC/acado_dummy_file.c
  ${acado_subleader_src}/acado_qpoases_interface.cpp
  ${acado_subleader_src}/acado_integrator.c
  ${acado_subleader_src}/acado_solver.c
  ${acado_subleader_src}/acado_auxiliary_functions.c
)

target_include_directories(subleader_mpc_lib PUBLIC 
  ${acado_subleader_src}/
  ${qpoases}
  ${qpoases}/SRC
  ${qpoases}/INCLUDE  
)

# Acado MPC solver interface for subleader
add_library(subleader_ctl ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(subleader_ctl subleader_mpc_lib ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(subleader_ctl_node nodes/subleader_ctl_node.cpp)
target_link_libraries(subleader_ctl_node subleader_ctl subleader_mpc_lib )

### UNIT TEST MPC SECTION
# 
# Create the MPC solver library
add_library(unittest_lib
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
  ${qpoases}/SRC/acado_dummy_file.c
  ${acado_unit_test_src}/acado_qpoases_interface.cpp
  ${acado_unit_test_src}/acado_integrator.c
  ${acado_unit_test_src}/acado_solver.c
  ${acado_unit_test_src}/acado_auxiliary_functions.c
)

target_include_directories(unittest_lib PUBLIC 
  ${acado_unit_test_src}/
  ${qpoases}
  ${qpoases}/INCLUDE
  ${qpoases}/SRC 
)

# Acado MPC solver interface for leader
add_library(unit_test_ctl ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(unit_test_ctl unittest_lib ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(unittest_ctl_node nodes/unittest_ctl_node.cpp)
target_link_libraries(unittest_ctl_node unit_test_ctl unittest_lib )

# ------------------------------------------------- #
#                                                   #
#                    GROUND TESTS                   #
#                                                   #
# ------------------------------------------------- #

### LEADER MPC SECTION
# 
# Create the MPC solver library
add_library(leader_mpc_lib_gnd
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
  ${qpoases}/SRC/acado_dummy_file.c
  ${acado_leader_src_gnd}/acado_qpoases_interface.cpp
  ${acado_leader_src_gnd}/acado_integrator.c
  ${acado_leader_src_gnd}/acado_solver.c
  ${acado_leader_src_gnd}/acado_auxiliary_functions.c
)

target_include_directories(leader_mpc_lib_gnd PUBLIC 
  ${acado_leader_src}/
  ${qpoases}
  ${qpoases}/INCLUDE
  ${qpoases}/SRC 
)

# Acado MPC solver interface for leader
add_library(leader_ctl_gnd ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(leader_ctl_gnd leader_mpc_lib_gnd ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(leader_ctl_node_gnd nodes/leader_ctl_node.cpp)
target_link_libraries(leader_ctl_node_gnd leader_ctl_gnd leader_mpc_lib_gnd )


### SUBLEADER MPC SECTION
# 
# Create the MPC solver library
add_library(subleader_mpc_lib_gnd
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
  ${qpoases}/SRC/acado_dummy_file.c
  ${acado_subleader_src_gnd}/acado_qpoases_interface.cpp
  ${acado_subleader_src_gnd}/acado_integrator.c
  ${acado_subleader_src_gnd}/acado_solver.c
  ${acado_subleader_src_gnd}/acado_auxiliary_functions.c
)

target_include_directories(subleader_mpc_lib_gnd PUBLIC 
  ${acado_subleader_src_gnd}/
  ${qpoases}
  ${qpoases}/SRC
  ${qpoases}/INCLUDE  
)

# Acado MPC solver interface for subleader
add_library(subleader_ctl_gnd ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(subleader_ctl_gnd subleader_mpc_lib_gnd ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(subleader_ctl_node_gnd nodes/subleader_ctl_node.cpp)
target_link_libraries(subleader_ctl_node_gnd subleader_ctl_gnd subleader_mpc_lib_gnd )

### UNIT TEST MPC SECTION
# 
# Create the MPC solver library
add_library(unittest_lib_gnd
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
  ${qpoases}/SRC/acado_dummy_file.c
  ${acado_unit_test_src_gnd}/acado_qpoases_interface.cpp
  ${acado_unit_test_src_gnd}/acado_integrator.c
  ${acado_unit_test_src_gnd}/acado_solver.c
  ${acado_unit_test_src_gnd}/acado_auxiliary_functions.c
)

target_include_directories(unittest_lib_gnd PUBLIC 
  ${acado_unit_test_src_gnd}/
  ${qpoases}
  ${qpoases}/INCLUDE
  ${qpoases}/SRC 
)

# Acado MPC solver interface for leader
add_library(unit_test_ctl_gnd ${controllers_src}/acado_wrapper.cpp)
target_link_libraries(unit_test_ctl_gnd unittest_lib_gnd ${catkin_LIBRARIES})

# Acado MPC solver node
add_executable(unittest_ctl_gnd_node nodes/unittest_ctl_node.cpp)
target_link_libraries(unittest_ctl_gnd_node unit_test_ctl_gnd unittest_lib_gnd )

if(ASAP_DEBIAN)
  install_launch_files()
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
  # Install libs
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
  # Install src directory
  install(DIRECTORY src/ DESTINATION share/${PROJECT_NAME}/src)
  install(TARGETS  
          unit_test_ctl_gnd unittest_lib_gnd
          unit_test_ctl unittest_lib
          subleader_mpc_lib_gnd subleader_ctl_gnd
          subleader_mpc_lib subleader_ctl
          leader_mpc_lib_gnd leader_ctl_gnd
          leader_mpc_lib leader_ctl
          DESTINATION lib)
  # Install scripts and nodes
  install(PROGRAMS nodes/ros_node_leader.py nodes/ros_node_subleader.py nodes/ros_node_unit_tests.py DESTINATION share/${PROJECT_NAME}/nodes PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE)
  install(TARGETS
          unittest_ctl_gnd_node
          unittest_ctl_node
          subleader_ctl_node_gnd
          subleader_ctl_node
          leader_ctl_node_gnd
          leader_ctl_node
          DESTINATION share/${PROJECT_NAME}/nodes PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE)
endif()
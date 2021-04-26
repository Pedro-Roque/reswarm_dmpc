cmake_minimum_required(VERSION 3.5.1)
project(reswarm_dmpc)

set(acados_include "/home/roque/Git/acados/include")
set(acados_lib "/home/roque/Git/acados/lib")
set(NMPC_controller ${PROJECT_SOURCE_DIR}/nodes/c_generated_code)

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
#   cfg/weightGenerationExample.cfg
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

catkin_python_setup()

# Set default include paths and directories
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${NMPC_controller}
#   ${NMPC_controller}/astrobee_model
#   ${NMPC_controller}/astrobee_cost
#   ${acados_include}
#   ${acados_include}/blasfeo/include/
#   ${acados_include}/hpipm/include/
#   ${acados_include}/acados/
#   ${acados_include}/acados_c/
#   ${acados_include}/qpOASES_e/
# )

# link_directories(${NMPC_controller} ${acados_lib})

# # NMPC node
# add_definitions( "-fPIC" ) # -O3
# add_library(casadi_lib 
#             ${NMPC_controller}/astrobee_model/astrobee_expl_ode_fun.c 
#             ${NMPC_controller}/astrobee_model/astrobee_expl_vde_forw.c
# )
# add_library(ocp_cost 
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_0_fun.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_0_fun_jac.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_0_fun_jac_hess.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_fun.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_fun_jac.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_fun_jac_hess.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_e_fun.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_e_fun_jac.c
#             ${NMPC_controller}/astrobee_cost/astrobee_cost_ext_cost_e_fun_jac_hess.c
# ) 
# add_library(ocp_solver 
#             ${NMPC_controller}/acados_solver_astrobee.c
#             ${NMPC_controller}/acados_sim_solver_astrobee.c)
# target_link_libraries(ocp_solver ocp_cost casadi_lib -lacados)

# add_executable(run_acados_compiled ${PROJECT_SOURCE_DIR}/nodes/run_acados_compiled.cpp)
# target_link_libraries(run_acados_compiled ocp_cost ocp_solver casadi_lib -lhpipm -lacados)

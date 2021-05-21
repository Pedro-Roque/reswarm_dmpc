#include "reswarm_dmpc/acado_mpc.h"

/*
Leader controller interface for ACADO.

Uses the same wrapper in AcadoMPC, linked against the controller
generated in reswarm_mpc/controllers/formation/acado/leader.

Spawns get_control and set_weights services, accessible through
ROS Service interface.
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "unit_test_1");
  ros::NodeHandle nh("");

  // Instantiate controller class
  AcadoMPC unit_test_1(nh);

  ros::spin();

  return 0;
}
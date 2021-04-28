#include "reswarm_dmpc/leader_ctl.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "leader_ctl_iface");
  ros::NodeHandle nh("");

  // Instantiate controller class
  LeaderMPC leader_ctl(nh);

  ros::spin();

  return 0;
}
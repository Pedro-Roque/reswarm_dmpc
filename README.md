# ReSwarm Distributed Nonlinear MPC - ROS Package
ROS Package for the ReSwarm Distributed MPC methods for the ReSWARM test sessions. 

Developed on and for ROS Kinetic.

### Usage:
1. Clone the repo into a `catkin_ws/src` directory 
2. Buiild the workspace with `catkin build` in the `catkin_ws` folder
3. Run the nodes `unit_test_<method>.py` to test different types of MPC strategies using `rosrun reswarm_dmpc unit_test_stabilization.py` or `rosrun reswarm_dmpc unit_test_tracking.py`. 

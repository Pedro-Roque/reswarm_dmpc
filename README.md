# ReSwarm Distributed Nonlinear MPC 
A package for Distributed Nonlinear MPC, based on CasADi, ACADOS and ROS middleware. 

This package provides system models and Model Predictive Controllers to control a team of [Astrobees](https://github.com/nasa/astrobee) free-flyers for both setpoint stabilization and trajectory tracking.

## Installation
This package depends on ROS Kinetic, but should be compatible with newer versions of ROS. It is assumed that the user previously installed ROS on his system. 

1. Install the [Astrobee](https://github.com/nasa/astrobee) simulator following the instructions for [General Users](https://nasa.github.io/astrobee/html/install-nonNASA.html)

2. Clone reswarm_dmpc to your catkin_ws and build the workspace
```
cd catkin_ws/src
git clone https://github.com/Pedro-Roque/reswarm_dmpc.git
cd reswarm_dmpc/
pip install -r requirements.txt
cd ../../
catkin build
source devel/setup.sh
```

3. Run the Astrobee simulator using the provided launch files
```
roslaunch reswarm_dmpc astrobee_sim.launch
```

4. Run a unit test!
```
roslaunch reswarm_dmpc unit_test_translation_interface.launch
```

5. Make sure that the robots are not in a faulty state by overriding it with
```
rostopic pub /honey/mgt/sys_monitor/state ff_msgs/FaultState '{state: 0}' & \
rostopic pub /bumble/mgt/sys_monitor/state ff_msgs/FaultState '{state: 0}'
```
6. At this point, the node should show "Sleeping..." as a ROS Info message. This means that the node is waiting to be started. To start the node, call the starting service for each robot with
```
rosservice call /honey/start "data: true"
```
and
```
rosservice call /bumble/start "data: true"
```

7. After a few seconds, the robots should be moving and the window where the unit test was launched will print information regarding the controller status, computational times, function costs, among other variables of interest.

## Acknowledgements 
A special thanks goes Bryce Doerr, Keenan Albee, Monica Ekal,  Brian Coltin and Rubén Ruiz, as well as to all the Astrobee Ops team, for their support in-view of the MPP ReSWARM test sessions and Astrobee Flight Software.
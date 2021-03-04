# ReSwarm Distributed Nonlinear MPC - ROS Package
ROS Package for the ReSwarm Distributed MPC methods for the ReSWARM test sessions. 

Developed on and for ROS Kinetic.

### Install Dependencies:
To install dependencies, please run:
```
python2 deps_install.py
```

### Usage with ROS:
1. Clone the repo into a `catkin_ws/src` directory 
2. Buiild the workspace with `catkin build` in the `catkin_ws` folder
3. Run the nodes `unit_test_<method>.py` to test different types of MPC strategies using `rosrun reswarm_dmpc unit_test_stabilization.py` or `rosrun reswarm_dmpc unit_test_tracking.py`. 

### Usage without ROS:
1. Clone this repo into a folder in your system (preferably `$HOME`)
```
cd $HOME
git clone https://github.com/Pedro-Roque/reswarm_dmpc.git
```
2. Add the `reswarm_dmpc/src` to your `$PYTHONPATH` with:
```
cd reswarm_dmpc
export $PYTHONPATH="$PYTHONPATH:$(pwd)/src"
```
3. Run the unit tests in the nodes folder. Example:
```
python2 nodes/unit_test_tracking.py
```
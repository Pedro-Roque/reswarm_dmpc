# ReSwarm Distributed Nonlinear MPC 
A package for Robust and Distributed Nonlinear MPC, based on CasADi, ACADOS and ROS frameworks and middleware. 

This package provides system models and Model Predictive Controllers to control an Astrobee free-flyer for both setpoint 
stabilization and trajectory tracking.

### Install Dependencies:
Required Python version: Python 2.7.17
Tested on: Ubuntu 18.04 , might work on most Linux-based OSs.
Python 3 compatibility: looked at, but no guarantees are provided.

To install dependencies, please run:
```
python2 deps_install.py
```

### Usage with a simple Python Environemnt:
1. Clone this repository into a folder in your system (preferably `$HOME`)
```
cd $HOME
git clone https://github.com/Pedro-Roque/reswarm_dmpc.git
```
2. Add the `reswarm_dmpc/src` to your `$PYTHONPATH` with:
```
cd reswarm_dmpc
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"
```
3. Run the unit tests in the nodes folder with:
```
cd nodes/
py.test -l test_astrobee.py
py.test -l test_mpc.py
```
**Note:** that these tests are also running using Github Actions at every push to the remote repository.

### Usage with ROS (Robot Operating System):
1. Clone the repo into a `catkin_ws/src` directory 
2. Buiild the workspace with `catkin build` in the `catkin_ws` folder
3. Run the nodes `unit_test_<method>.py` to test different types of MPC strategies using `rosrun reswarm_dmpc unit_test_stabilization.py` or `rosrun reswarm_dmpc unit_test_tracking.py`. 

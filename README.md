# uneven_planner

## Quick Start

### Step One:

Install the requirements.

**gazebo plugins**: (use ros noetic with Ubuntu20.04 as an example)

```
sudo apt-get install ros-noetic-robot-state-publisher*
sudo apt-get install ros-noetic-joint-state-controller*
sudo apt-get install ros-noetic-controller*
sudo apt-get install ros-noetic-velocity-controllers*
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-position-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-hector-gazebo
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-joint-state-controller
sudo apt-get install ros-noetic-position-controllers
```

**osqp-0.6.2 and osqp-eigen v0.8.0 for mpc controller:**

Firstly, go to [website of OSPQ](https://github.com/osqp/osqp/releases) and download `complete_sources.zip` from the Assets of `0.6.2`. Then unzip the code,

```
cd osqp
mkdir build && cd build
cmake ..
make
sudo make install
```

Go to [website of osqp-eigen](https://github.com/robotology/osqp-eigen/releases) and download `Source code.zip` from the Assets of `osqp-eigen v0.8.0`. Then unzip the code,

```
cd osqp-eigen-0.8.0
mkdir build && cd build
cmake ..
make
sudo make install
```

**NOTE:** We may have forgotten other dependencies 😟, sorry!

### Step Two:

Build the project:

```
git clone https://github.com/ZJU-FAST-Lab/uneven_planner.git
cd uneven_planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Step Three:

Select different scenes to run by the scripts (map_mountain.dae is too large, can't be upload to github)

* hill: `./hill.sh`
* desert: `./desert.sh`
* volcano: `./volcano.sh`
* forest: `./forest.sh`

If this is the first time you've run the scene, you may need to wait a few moments for constructing the mapping $\mathscr{F}:SE(2)\rightarrow\mathbb{R}\times\mathbb{S}_+^2$. This could be:

<img src='figures/waiting.png'>

When you see the point cloud in the Rviz like in the hill scene below, you can use `2D Nav Goal` to choose the planning target.

<img src='figures/pc_done.png'>

**NOTE:**

* Due to the simplicity of the model, wheel slippage may occur in the simulation, resulting in poor trajectory tracking accuracy.
* In the forest environment, shrubs are solid triangular meshes which may cause the robot to get stuck while moving in the simulation.

## Citing

The method used in this software are described in the following paper (available [here](https://arxiv.org/abs/2309.06115))

Title: An Efficient Trajectory Planner for Car-like Robots on Uneven Terrain

[Video for the IROS submission](https://www.youtube.com/watch?v=BPfoun_vQ4I)

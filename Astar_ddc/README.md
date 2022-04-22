# AStar- with non holonomic constraints.

## Authors: 
### Gokul Hari - 117430646
### Aswath Muthuselvam - 
# Results
![video](./assets/simulation_video.gif)


# File structure.
```
├── arena.py
├── assets
│   ├── simulation_video.gif
│   ├── simulation_video.mp4
│   └── test.py
├── Astar_ddc.py
├── README.md
└── utils.py
```

# Part2

# ROS package.

Refer the folder `./ros` for the ros package `astar_ddc`. Paste the ros package in your src folder of catkin workspace.

### Dependancies. 
We need turtle bot and navigation_msgs
run the following 

```
cd ~/ros_catkin_ws/src

git clone https://github.com/ros-planning/navigation_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
make sure to paste this in `.bashrc` -   export TURTLEBOT3_MODEL=burger 

Run `cd ~/catkin_ws && catkin_make`

The following scripts are in the package's folder `astar_ddc/src` 
-  `Astart_ddc.py`
- `arena.py` 
-  `utils.py` 
- `open_loop.py` 
- `closed_loop.py`

The node `publisher.py` is used to call the Astar planner to make the plan and obtain the waypoints. 
We have an `open loop publisher` that can blindly apply the actions to the model, we inferred that these results were not suitable.
We also have a `closed loop publisher ` that can set the pose in each waypoint to be reached and  the model reaches it.
This is done with the help of actionlib and move_base.

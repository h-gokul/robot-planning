
## Install dependencies:
```
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-actionlib-tools
```

- Get the list of action servers:
```
rostopic list | grep -o -P '^.*(?=/feedback)'
```
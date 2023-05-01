# Setup instructions
## Creating a catkin workspace
```
sudo apt-get update
sudo apt-get install -y \
  python3-catkin-tools
cd ~/ros_ws
catkin init
```

# Install ROS packages (this step may take few minutes)
Assuming the packages are already cloned into `~/ros_ws/src`, then the dependencies can be installed using
```
rosdep update
rosdep install -i --from-path /home/ros/ros_ws/src --rosdistro melodic -y
```

# Building the package
```
# Build package
catkin build

# Source and run package
source devel/setup.bash
roslaunch vention_example pick_and_place_demo.launch
```

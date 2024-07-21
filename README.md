# SLAM simulation using Turtlebot3

This repository contains a custom ROS package named mrob_highlight_controller. This package facilitates the creation of a 2D map of an unknown environment using the slam_gmapping node from the gmapping package. It also enables navigation to a specific point using the move_base node from the move_base package and performs localization using the amcl node.

### Demo Video
You can watch the demo video by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/rviz.png)](https://youtu.be/E09mhgvatxQ)

## Map creation
The map of any unknown environment can be created using the gmapping package. To create the map, first run the mapping launch file, and then control the robot using the teleop keyboard to explore the environment and generate the map.

### Note 
1.  Map is already created and saved inside the maps folder of the package, so you don't have to recreate the map.
2.  Details about cloning the repository are given at the end of this **readme file**

To launch the mapping.launch file (you should first navigate inside the launch folder of the package and then, use the following command): 
```bash
roslaunch mapping.launch
```

![Map](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/map_creation.png)

Keep exploring the environment until the entire area is mapped.

![Map](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/map.png)

Once the map is created, create a new folder where you can save the maps:
```bash
mkdir maps
```
The map can be saved using the following command:
```bash
rosrun map_server map_saver -f turtlebot3_world_map
```

## Navigation
Once the map is saved, you can start navigating in the environment. To launch the navigation.launch file (you should first navigate inside the launch folder of the package and then, use the following command): 
```bash
roslaunch navigation.launch use_gazebo:=true
```
#### Note: 
use_gazebo:=true should be added to open the gazebo simulation.

### Important
1.  To confirm the robot's pose, you need to use the **2D Pose Estimate** feature. Click on the 2D Pose Estimate button from the Toolbar, then click on the map where you see the robot is located. This sets the initial position and orientation of the robot on the map. This step is crucial because it helps the robot understand where it is in relation to the map, ensuring accurate navigation and path planning.

2.  The robot won't move, because you have to specify target position first. To set goal position for robot click on **2D nav goal** button from Toolbar, then click a place in **Visualization window**, that will be destination for your robot. Observe as path is generated (you should see a line from your robot pointing to direction) and robot is moving to its destination. 

![RVIZ](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/rviz.png)

![Gazebo](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/gazebo.png)

# Create Ros Workspace
Open shell and execute the following commands:
```bash
mkdir slam_repo
```
```bash
cd slam_repo
```
# Clone the repository
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/turtlebot_slam_ws.git
```
roscore must be running in order for ROS nodes to communicate. Open a new terminal and run roscore with this command:
```bash
roscore 
```
Run the below commands in root folder of the workspace
```bash
catkin_make 
```
```bash
source devel/setup.bash 
```
Press Enter and navigate to the launch folder inside the package
```bash
roslaunch navigation.launch use_gazebo:=true
```

## Note
Symbolic link of **robotics_course** and **teleop_twist_keyboard** packages can be created inside the **src** folder of the workspace, if you have saved these packages in another folder (let's say name of the folder is **git**, and it is present on **Desktop**)
Symbolic link can be created with this command:
```bash
ln -s ~/Desktop/git/robotics_course
```
Remember: The above command should be executed inside the folder where you want to create the symbolic links (in our case inside the src folder)


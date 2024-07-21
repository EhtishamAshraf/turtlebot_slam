# SLAM simulation using Turtlebot3

This repository contains a custom ROS package named mrob_highlight_controller. This package facilitates the creation of a 2D map of an unknown environment using the slam_gmapping node from the gmapping package. It also enables navigation to a specific point using the move_base node from the move_base package and performs localization using the amcl node.

## Map creation
The map of any unknown environment can be created using the gmapping package. To create the map, first run the mapping launch file, and then control the robot using the teleop keyboard to explore the environment and generate the map.

![Map](https://github.com/EhtishamAshraf/turtlebot_slam/blob/main/src/mrob_highlight_controller/maps/autorace_map.png)

## Below are the results
### Creating a circle 
![Circle](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/circle.png)

### Creating a square 
![Square](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/square_start.png)
![Square](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/square.png)

### Creating a rectangle 
![Rectangle](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/rectangle.png)

### Creating a triangle 
![Triangle](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/triangle.png)

### Moving towards goal 
![Goal](https://github.com/EhtishamAshraf/ros_turtle_shapes/blob/main/goal.png)

# Create Ros Workspace
Open shell and execute the following commands:
```bash
mkdir my_ros_ws
```
```bash
cd my_ros_ws
```
```bash
mkdir src
```
Run the below command in root folder of the workspace
```bash
catkin_make 
```
Navigate to the src folder and clone the repository
```bash
cd src 
```
# Clone the repository
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/ros_turtle-shapes.git
```
Run the command below, to make the python file executable. Run the command in the src folder (where the python file is present)
```bash
chmod +x turtle_shapes.py
```
roscore must be running in order for ROS nodes to communicate. Open a new terminal and run roscore with this command:
```bash
roscore &
```
Press Enter and write the command below to open turtlesim : (Assuming that turtlesim is already installed!) 
```bash
rosrun turtlesim turtlesim_node 
```
Open a new shell windown, navigate to the root of the workspace and then run the following the commands in sequence:
Command to build the package
```bash
catkin_make
```
Command to add environment variables needed by ROS
```bash
source devel/setup.bash
```
Run the code:
```bash
rosrun ros_turtle-shapes turtle_shapes.py
```

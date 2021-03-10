# PDE4422_Group_Project
This repository contains a Group Project for the PDE4422 module.

The developers are MSc Robotics students from Middlesex University London: Aleksei Korotchenko, Caleb Mukaswanga, Meenakshi Loganathan, Monitrice Turner, Swetang Khatri.

The project presents a mobile robot based on TurtleBot3 with added Robot Arm, which can navigate in the created environment using the Navigation Stack, travel according to set points and move objects using the gripper. For this purpose, a robot is created based on the TurtleBot3 model, to which an arm capable of moving objects is added, an environment is created, sensors, Python program and The Navigation Stack are developed to navigate the robot.

## Commands to start work
1. Create workspace:
```
mkdir -p ~/group_project_ws/src
```
2. Install packages:
```
sudo apt-get install ros-melodic-moveit-msgs
sudo apt-get install ros-melodic-moveit-core
sudo apt-get install ros-melodic-moveit-ros-planning
sudo apt-get install ros-melodic-moveit-ros-planning-interface
```
3. Clone Group Project repository:
```
cd ~/group_project_ws/src/
git clone https://github.com/aleksei089/PDE4422_Group_Project.git .
```
4. Make:
```
cd ~/group_project_ws/
catkin_make
```
5. Set Waffle model. Open the bashrc file:
```
gedit ~/.bashrc
```
Add lines at the bottom of the file:
```
export TURTLEBOT3_MODEL=waffle_pi
source devel/setup.bash
```
Save the file and close it. Reload .bashrc:
```
source ~/.bashrc
```
## Launch commands
1. Launch Gazebo (don't forget to start the simulation in it!!! |>):
```
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```
2. Launch Controller (in a new terminal):
```
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
3. Launch Manipulator Control from Window (in a new terminal):
```
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
3. **OR** Launch Manipulator Control from MoveIt!/RViz (in a new terminal):
```
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
4. To Drive a TurtleBot3 (in a new terminal):
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
5. Launch Python File for Automatic Grip (better when the TurtleBot3 is stopped) (in a new terminal):
```
rosrun turtlebot3_manipulation_moveit_config test1.py
```
## Uploading files to GitHub
Jump to the PDE4422_Group_Project folder:
```
cd ~/group_project_ws/src/PDE4422_Group_Project/
```
Uploading (in "release v?" symbol "?" indicates a release version number):
```
git add .
git commit -m "release v?"
git push -u origin master
```
Enter username and password.

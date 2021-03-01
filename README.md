# PDE4422_Group_Project
This repository contains a Group Project for the PDE4422 module.

The developers are MSc Robotics students from Middlesex University London: Aleksei Korotchenko, Caleb Mukaswanga, Meenakshi Loganathan, Monitrice Turner, Swetang Khatri.

The project presents a mobile robot based on TurtleBot3 with added Robot Arm, which can navigate in the created environment using the Navigation Stack, travel according to set points and move objects using the gripper. For this purpose, a robot is created based on the TurtleBot3 model, to which an arm capable of moving objects is added, an environment is created, sensors, Python program and The Navigation Stack are developed to navigate the robot.

## Commands to start work
1. Create workspace:
```
mkdir -p ~/group_project_ws/src
```
2. Clone Group Project repository:
```
cd ~/group_project_ws/src/
git clone https://github.com/aleksei089/PDE4422_Group_Project.git
```
3. Make:
```
cd ~/group_project_ws/
catkin_make
```
4. Set Waffle model. Open the bashrc file:
```
gedit ~/.bashrc
```
Add line at the bottom of the file:
```
export TURTLEBOT3_MODEL=waffle_pi
source devel/setup.bash
```
Save the file and close it. Reload .bashrc:
```
source ~/.bashrc
```
## Launch commands
Launch RViz:
```
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
Launch Gazebo (in a new terminal):
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
To move TurtleBot3 (in a new terminal):
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
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
git remote add origin https://github.com/aleksei089/PDE4422_Group_Project.git
git push -u origin master
```
Enter username and password.

# rse-nd-ekf
Course projects repository for the Udacity Robotics Software Engineer Nanodegree program.

This project contains the following Catkin packages :
* main: A launch file to launch all packages requires with respective parameters to run EKF on a simulated robot in a gazebo world.

## Installation
Clone this repository and other required packages in **src** folder in your catkin workspace (considering it is located at `~/catkin_ws`)
```
cd ~/catkin_ws/src
git clone https://github.com/rodriguesrenato/rse-nd-ekf.git
cd rse-nd-ekf
git clone https://github.com/udacity/robot_pose_ekf 
git clone https://github.com/udacity/odom_to_trajectory
git clone https://github.com/turtlebot/turtlebot
cd ~/catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_teleop
catkin_make
source devel/setup.bash
```
## Usage
Supposing your catkin workspace is located in ~/
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Use the `main.launch` file to launch all packages together.
```
roslaunch main main.launch
```
## License
The contents of this repository are covered under the MIT License.

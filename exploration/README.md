# AI maze solver using fuzzy
Open source implementation of an AI maze solver in python using fuzzy


## Requirements
+ Linux/Windows Subsystem for Linux (WSL)
+ Ubuntu 22.04.3
+ ROS2 Ubuntu (Debian packages) https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


## Built with
+ python 3.10.12
+ numpy 1.23.5 
+ scikit-fuzzy 0.4.2 
+ rclpy 3.3.11
+ math
+ threading
+ TurtleBot3 Humble https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/


## Usage
To run the maze solver, first source the setup file:
```
source /usr/share/gazebo/setup.sh
```
Then select turtlebot:
```
export TURTLEBOT3_MODEL=waffle
```
Launch Gazebo maze map (see maze repository):
```
ros2 launch turtlebot3_gazebo maze.launch.py
```
Start main file to start AI maze solver control of the turtlebot:
```
./main.py
```


## License
Distributed under the MIT license.


## Contact
Carl Larsson - cln20001@student.mdu.se


Research Track I - Assignment 2
================================

This project shows up a mobile robot moving in a 3D space, which has to reach a desired position by avoiding obstacles. This happens in a virtual environment, managed by the standard virtual simulator of ROS, called Gazebo.

To achieve this, the project requires to implement:
- (a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom.
- (b) A service node that, when called, prints the number of goals reached and cancelled;
- (c) A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed.

Installing and running
----------------------
To run the program, we have to install xterm:
```bash
$ sudo apt-get install xterm
```
and SciPy
```bash
$ sudo apt-get install python3-scipy
```
Go inside the src folder of your ROS workspace and clone the assignment folder:
```bash
$ git clone
```
Then, from the root directory of your ROS workspace run the command:
```bash
$ catkin_make
```
You can run `$ roscore`  in a terminal or skip it. Anyway, it will be runned automatically. Run the string below to start the programme:
```bash
$ roslaunch assignment_2_2022 assignment2.launch
```

## Troubleshooting
Check your python version running:
```bash
$ python --version
```
If it appears Python 3, there is no problem. If appears Python 2, run this:
```bash
$ sudo apt-get install python-is-python3
```


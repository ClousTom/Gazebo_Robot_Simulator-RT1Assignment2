Research Track I - Assignment 2
================================

This project shows up a mobile robot moving in a 3D space, which has to reach a requested target coordinates avoiding obstacles. This happens in a virtual environment, managed by the standard virtual simulator of ROS, called Gazebo.

![default_gzclient_camera(1)-2023-01-27T15_23_29 351549](https://user-images.githubusercontent.com/117213899/215110545-c90c0ec1-ce38-4ce9-86e4-545e46050bc1.jpg)

Installing and running
----------------------
To run the program, we have to install xterm:
```bash
$ sudo apt-get install xterm
```
and SciPy:
```bash
$ sudo apt-get install python3-scipy
```
Go inside the src folder of your ROS workspace and clone the repository folder:
```bash
$ git clone https://github.com/S5630055/ResearchTrack1---Assignment_2
```
Then, from the root directory of your ROS workspace, run the command:
```bash
$ catkin_make
```
You can run `$ roscore`  in a terminal or skip it. Anyway, it will be runned automatically. Run the string below to start the programme:
```bash
$ roslaunch assignment_2_2022 assignment2.launch
```

Troubleshooting
----------------------
Check your python version running:
```bash
$ python --version
```
If it appears Python 3, there is no problem. If it appears Python 2, run this:
```bash
$ sudo apt-get install python-is-python3
```

Nodes
----------------------
Inside `~/<your ros workspace folder>/src/assignment_2_2022/scripts/` there are 6 python files:

1. `bug_as.py`: action server node receiving the requested position from the client and calling the necessary services to bring the robot to the required position;
2. `input.py`: action client node responsible for asking the user to enter the coordinates X and Y of the final destination that the robot has to reach, or to delete them. Then, it publishes the robot position and speed as a custom message on the /_position_velocity_ topic, based on the values of the /_odom_ topic.
3. `printer.py`: node printing on the terminal the distance of the robot from the target position and its average speed. These parameters are taken from the _/position_velocity_ topic as a custom message.
4. `go_to_point_service.py`: implementation of a service node. When called, it moves the robot to the requested position.
5. `wall_follow_service.py`: implementation of a service node. When called, it allows the robot to move around an obstacle (in our case a wall).
6. `service.py`: it is a service node. When called, it prints the number of successful reached targets and the number of cancelled targets.

Flowchart (`input.py`)
----------------------
![flowchart](https://user-images.githubusercontent.com/117213899/215106562-20902234-1fe8-4b8e-b1c5-32a7173657ff.png)

Possible improvements
----------------------
Here are some ideas for future improvements:
- By indicating the size of the arena, it would be possible to set a range of possible target coordinates;
- A graphic indicator could be added at the point corresponding to the entered coordinates to be reached.

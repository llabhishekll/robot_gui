# Robot Info (Node)

Target is to develop a user-friendly graphical interface that even those with little or no previous experience in robotics can use to interact with the robots remotely. The system uses several ROS nodes communicating with each other and the graphical interface build using [cvui](https://dovyski.github.io/cvui/) library. The system involves tree important functions (nodes) to work properly:

- **Robot GUI** : GUI application functions as a ROS node to communicate and displays data on the user's screen
- **Robot Info** : provides relevant information about the robot, such as the robot name and the current status of its systems
- **Distance Tracker** : calculates the distance traveled by the robot using the odometry data and work as service.

![ros](https://img.shields.io/badge/ROS-noetic-red) ![cpp](https://img.shields.io/badge/cpp-11+-blue)

## Structure

```text
.
├── CMakeLists.txt
├── include
│   └── robot_gui
│       ├── cvui.h
│       └── robot_gui_backend.h
├── package.xml
├── README.md
└── src
    ├── robot_gui_backend.cpp
    └── robot_gui_node.cpp
```

## Setup

#### Distribution

Use docker for quick-start (for both ROS1 or ROS2):

```sh
# using docker for ROS1
$ docker run -ti --rm --name local-ros-noetic ros:noetic
```

```sh
# using docker for ROS2
$ docker run -ti --rm --name local-ros-humble ros:humble
```

#### Build (Package)

Now, create a catkin workspace, clone the package:

```sh
# setup directory
$ mkdir ~/catkin_ws/src/
$ git clone <repo_name> ~/catkin_ws/src/
```

Install the required packages (dependency) mentioned in `package.xml` using `apt`:

```sh
# check if package is available
$ rospack list
$ rosnode list
```

```sh
# update path to installed packages
$ export ROS_PACKAGE_PATH='/home/user/catkin_ws/src:/opt/ros/noetic/share'
```

To build locally or inside docker use the following commands:

```sh
# execute build
$ cd ~/catkin_ws & catkin_make
$ source devel/setup.bash
```

## Robot (MiR100)

The `MiR100` is autonomous mobile robots that quickly automates internal transportation and logistics. The `MiR100` is four wheel robot which can run up-to `1.5 m/s` (max speed) and transfer up `100 Kg` payload.

To download the robot environment clone the repository:

```sh
git clone https://bitbucket.org/theconstructcore/advanced_cpp_auxiliary_pkgs.git ~/catkin_ws/src/
```

To launch the robot in gazebo simulation (by default, physics is in paused state on simulation start):

```sh
roslaunch mir_gazebo mir_maze_world.launch
```

![4](./.assets/4.png)

## Node

The node `robot_gui_node` defined in `robot_gui_node.cpp` initialize **RobotGUI** class object from `robot_gui_backend.cpp`, which utilize four subscribes, two service clients and one publisher to provide interface which is built using `cvui`.

- **General Info Area** : display the messages published into the `robot_info` topic.
- **Teleoperation Buttons** : increasing and decreasing the speed in the x-axis direction and the rotation on the z-axis.
- **Current Velocities** : display the current speed send to the robot via the `cmd_vel` topic as "Linear Velocity" and "Angular Velocity".
- **Robot Position** : subscribe to `/odom` and display the current x, y, z position to the interface.
- **Distance Traveled** : create a button that calls the `/get_distance` service and displays the response message to the screen.

<img src="./.assets/7.png" alt="7" style="zoom:50%;" />

To run the `robot_gui_node` node open two terminals, source and execute the following commands:

```sh
# terminal 1
$ roslaunch mir_gazebo mir_maze_world.launch
```

```sh
# terminal 2
$ rosrun robot_info agv_robot_info_node
```

```sh
# terminal 3
$ rosrun distance_tracker_service distance_tracker_service
```

```sh
# terminal 4
$ rosrun robot_gui robot_gui_node
```

![7](./.assets/7.gif)

## Specifications

#### Class Diagram

```mermaid
classDiagram
 class RobotGUI
    RobotGUI : +vector~string~ robot_info
    RobotGUI : +double cmd_vel_linear_x
    RobotGUI : +double cmd_vel_angular_z
    RobotGUI : +double odom_x
    RobotGUI : +double odom_y
    RobotGUI : +double odom_z
    RobotGUI : +string distance
    RobotGUI : ...
    RobotGUI : +publisher_call_linear()
    RobotGUI : +publisher_call_angular()
    RobotGUI : +publisher_call_halt()
    RobotGUI : ...()
```

#### Distance Service

The service `/get_distance` is part of `advanced_cpp_auxiliary_pkgs` under the package name of `distance_tracker_service`, reset functionality is dependent on modification of the package to include another service named under `/reset_distance` which set variable `distance_` equal to zero upon client call.

## Roadmap

- [x] Part 1 : Create `robot_info_node` and `agv_robot_info_node`.

- [x] Part 2 : Create `/get_distance` service node.

- [x] Part 3 : Create `robot_gui_node` for robot control.

See the [open issues](https://github.com/llabhishekll/) for a full list of proposed features (and known issues).

## Tools

System tool/modules used for project development.

- `Applications` : [vs-code](https://code.visualstudio.com/), [ros-extensions](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) and [docker-desktop](https://docs.docker.com/get-docker/).
- `ROS` : [ros-docker-images](https://hub.docker.com/_/ros/) (`humble`, `noetic`) or [build-source](https://www.ros.org/blog/getting-started/).

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

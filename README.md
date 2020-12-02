# Universal Robot Demo

__Installation__

There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.


___Using apt (Ubuntu, Debian)___

On supported Linux distributions (Ubuntu, up to 16.04 (Xenial), `i386` and `amd64`) and ROS versions:

```
sudo apt-get install ros-$ROS_DISTRO-universal-robot
```

replace `$ROS_DISTRO` with `hydro`, `indigo`, `kinetic` or 'melodic', depending on which ROS version you have installed.


___Building from Source___

```
mkdir -p catkin_ur/src
cd catkin_ur/src/
git clone https://github.com/Michal-Bidzinski/universal_robots_demo.git
cd ..
catkin_make
```

___How to run___

To start Gazebo simulation, MoveIt! and Rviz:

```
source devel/setup.bash
roslaunch move_robot simulator.launch
```

Now you can plan trajectory by gui in Rviz.

To plan trajectory from script, in  new terminal:

```
source devel/setup.bash
rosrun move_robot move_ur3.py
```




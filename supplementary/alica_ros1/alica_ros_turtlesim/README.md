# Turtlesim Tutorials

## 1. Overview

This collection of tutorials is an extension of the [turtlesim ROS package](http://wiki.ros.org/turtlesim). By following these tutorials, you will learn

- some of the core concepts of the ALICA language
- how to achieve multi-agent collaboration with the ALICA framework

In this tutorial, you will create an application as shown in the picture below. The ALICA engine will assign the `Leader` task to one turtle and the `Follower` task to the other turtles. Further, the ALICA engine will move the turtles to their goal positions based on distance constraints.

![overview](./doc/overview.png)

## 2. Prerequisite

It is assumed that you are running the Ubuntu 20.04.5 LTS. these tutorials may work on other distributions but may require additional work or cleverness.

You need to be familiar with following topics and tools:

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

These tutorials will assume you have a working and configured ROS workspace and are able to build ros packages.

Begin by cloning the alica repository into your workspace and building the turtlesim package and its dependencies

```
cd ${CATKIN_WS}/src
git clone https://github.com/rapyuta-robotics/alica.git
apt-get update
apt-get install -y python3-catkin-tools libboost-all-dev
rosdep install -y --from-paths . --ignore-src --skip-keys='rclcpp ament_cmake' --rosdistro noetic
catkin build alica_ros_turtlesim
source ${CATKIN_WS}/devel/setup.bash
```

Additionally, you will need to set up the ALICA plan designer. However, lets start out by running some of the tutorials.

## 3. Running tutorials

Make sure you have build

```bash
cd catkin_ws
source /opt/ros/$(ls /opt/ros)/setup.bash
catkin build alica_ros_turtlesim
source ./devel/setup.bash
```

For each tutorial, you need to first launch the turtlesim environment

`roslaunch alica_ros_turtlesim env.launch`

The environment is a simulator node allowing us to spawn and visualize turtles, but to control the turtles we need to launch an alica application.  
Tutorial applications can be launched with:
`roslaunch alica_ros_turtlesim turtle.launch master_plan:=<TUTORIAL_NAME> turtles:=<NUM_TURTLES>`

The master_plan argument tells the ALICA application which plan to use as the master plan.
Each tutorial has its own master plan.
We will learn more about this when actually going through the tutorials.

### 3.1 RotatingTurtleTutorial

This is a basic application where a ALICA controls a turtle to rotate in place.
Make sure you have the simulation running, and launch the alica application with

`roslaunch alica_ros_turtlesim turtle.launch master_plan:=RotatingTurtleTutorial`

You should see a turtle rotating in the center of the screen as below:

TODO (picture)

### 3.2 MoveSequenceTutorial

In this tutorial the turtle will move to the four corners of the map in a sequence.
Make sure you have the simulation node running, and launch the alica application with

`roslaunch alica_ros_turtlesim turtle.launch master_plan:=MoveSequenceTutorial`

TODO (picture, or ideally gif)

Moreover, in this tutorial a topic is available to tell the turtle to change behavior.
The turtle also can do the rotate behavior from the previous behavior.  
Publishing on the turtle's `rotate` topic will toggle the turtle to rotate in place instead.

`rostopic pub turtle1/rotate std_msgs/Empty "{}"`

Publishing on the `move` topic will then toggle the behavior back so that the turtle moves between corners.

`rostopic pub turtle1/move std_msgs/Empty "{}"`

When toggling, you may notice that the turtle always starts from the bottom left corner regardless of how far it proceeded before rotating.
This will be explained more in the tutorial.

## 3.3 RandomMoveTutorial

In this tutorial the turtle will continuously choose a random point on the map, move there, then pause for a short period of time before choosing a new random point and repeating.  
We run this in the same way as with the previous tutorials, but lets spawn more than one turtle this time

`roslaunch alica_ros_turtlesim turtle.launch master_plan:=MoveSequenceTutorial turtles:=3`

TODO: picture/gif

## 3.4 FourCornersTutorial

This tutorial requires multiple turtles to be meaningful, and is meant to be run with four turtles
Each turtle will try to go to one corner of the map.
The corners are not fixed.
ALICA communication ensures that each turtle goes to a different corner.
Additional turtles (beyond 4) will fail to assign to a turtle and will sit in place instead.

The turtles can be launced with

`roslaunch alica_ros_turtlesim turtle.launch master_plan:=FourCornersTutorial turtles:=4`

TODO: Randomize start locations using TeleportToRandomPosition behavior, then use utility function to make turtles go to the closest corner, so that we can see that each turtle will sometimes go to a differnet corner

TODO: picture/gif, but maybe we finalize behavior first.

## 3.5 SurroundLeaderTutorial

In this tutorial one turtle will decide to be the leader and move to the center.
The other turtles will attempt to make an evenly spaced ring around the leader.
The turtles can be launched with

`roslaunch alica_ros_turtlesim turtle.launch master_plan:=FourCornersTutorial turtles:=5`

Feel free to try launching with more turtles (but be careful not to kill your CPU).

## 4. Setup of the ALICA Plan Designer

Now we want to try implementing these tutorials ourselves, but first we need to setup the plan designer.

The [ALICA Plan Designer](https://github.com/rapyuta-robotics/alica/tree/devel/supplementary/alica_designer_runtime) is a user interface to design applications with the ALICA framework.

The Plan Designer is a web application and can be started by using docker-compose.

### 4.1 Starting the plan designer

Navigate to alica/supplementary/alica_designer_runtime/ and execute the following command:

```
docker-compose up
```

You can open the Plan Designer with a browser of your choice by visiting the url
http://localhost:3030.

When you start the Plan Designer the first time, its main window should look like this:

![Empty Plan Designer](doc/Empty-PlanDesigner.png)

### 4.2 (Optional) Github integration

The ALICA plan designer can be connected to your github account to allow importing from or exporting directly to github branches.

More instructions are available in the plan designer's readme.

## 5. Tutorials

Each tutorial depends on having done the previous tutorials.

It may be possible to skip, but only if you know what you are doing.

[Tutorial 1: RotatingTurtleTutorial](docs/rotating_turtle.md)

[Tutorial 2: MoveSequenceTutorial](docs/move_sequence.md)

[Tutorial 3: RandomMoveTutorial](docs/random_move.md)

[Tutorial 4: FourCornersTutorial](docs/four_corners.md)

[Tutorial 5: SurroundLeaderTutorial](docs/surround_leader.md)

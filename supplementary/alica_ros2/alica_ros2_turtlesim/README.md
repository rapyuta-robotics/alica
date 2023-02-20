# ROS 2 Turtlesim Tutorial

## 1. Overview

This tutorial is a ROS 2 version of the Alica ROS Turtlesim, with support for ROS Galactic and Humble. See the ROS 1 Turtlesim package for details on how the turtlesim and Alica interact.

## 2. Setup of the Colcon Workspace

Use the appropriate version of Ubuntu for your ROS Distro (20.04 for Galactic, 22.05 for Humble) and run the following.

1. Check out the required repositories:

```
git clone https://github.com/rapyuta-robotics/alica.git
```

The repository may be cloned into a different directory if you wish, but this is not required.

2. If you wish to rebuild the plan from scratch, follow the steps 4.2 - 7 in the ROS1 turtlesim README. To quickly run the turtlesim, proceed to the next steps.

## 3. Build and Run

### 3.1 Build

Build the ROS 2 packages using Colcon.

```
source /opt/ros/[YOUR DISTRO HERE]/setup.bash
colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests
source install/setup.bash
```

### 3.2 Run

Run application with ros2 launch.

- Launch turtlesim
  `ros2 launch alica_ros2_turtlesim env.launch.xml`
- Turtle node (you can launch multiple turtles by setting the `turtles` launch arg)
  `ros2 launch alica_ros2_turtlesim turtle.launch.xml turtles:=2`
- Start moving.
  `ros2 topic pub /init std_msgs/msg/Empty`

## 4. Troubleshooting

### 4.1 Unknown quantifier type encountered

If you get the following error message when starting a turtle
`Unknown quantifier type encountered!`, have a look at the
file `etc/plans/Move.pml` and make sure the value of the field
`quantifierType` is set to `"all"`.

## Tutorial
### 1. Overview
This tutorial is extension of the [turtlesim ROS package](http://wiki.ros.org/turtlesim). By following this tutorial, you will learn 
- the core concepts of the ALICA language
- how to achieve multi-agent collaboration with the ALICA framework

In this tutorial, you will create an application as shown in the picture below. The ALICA engine will assign the “Leader” task to one turtle and the “Follower” task to the other turtles. Further, the ALICA engine will move the turtles to their goal positions based on distance constraints.

![overview](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/overview.png)
![alica_ros_turtlesim](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/alica_ros_turtlesim.gif)

### 2. Prerequisite
You need to be familiar with following topics and tools:
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

### 3. The ALICA Language - Basics and Core Concepts
We will only give you a brief explanation on the ALICA core concepts. For the interested reader, we recommend to consider the resources on the [ALICA Homepage](https://www.uni-kassel.de/eecs/fachgebiete/vs/research/alica.html) of the Distributed Systems Department of the University of Kassel for more detailed information in a series of related publications.
##### Plan
A plan is a state machine in tree structure. Plans can include plans and states and each state has can include`Behaviour`s. The ALICA engine assigns entrypoints of the  plan tree to the agents, e.g., robots based on `Role`, `Task`, `Constraints` and `Utility function`.
The ALICA engine manages state transitions based on the developers code. The ALICA plan designer generates method stubs that a developer will fill with state transition logic. The developer can create plans using the ALICA plan designer.

##### Behaviour
The developer can write robot behaviours in C++ for each state. The ALICA plan designer generates method stubs and the developer implements the behaviour logic in these stubs. In this tutorial, there are the `Go2RandomPosition` and `GoTo` behaviour.
The turtles are teleported to random position with `Go2RandomPosition` and they go to their target position with  the `GoTo` behaviour.

##### Role

A role is a task preference of the agent and it describes physical difference among agents , e.g., differences between a robotic arm and an AGV. In this tutorial all agent have same role:  `Turtle`

##### Task
A task is assigned to an agent based on the `Role` of that agent and based on the `Utility function` of the plan. A task identifies an entry point of a state machine. The ALICA engine realises multi-agent collaboration by assigning tasks to agents. In this tutorial, there are the `Leader` and the `Follower` task. One turtle is assigned the `Leader` task and it moves to the centre. The other turtles are assigned the `Follower` task and they align in a circle.
##### Constraints
Developers can set constraints to plans. The ALICA engine can solve constraints and return corresponding answers. In this tutorial, the turtles align in a circle defined by distance constraints.

##### Worldmodel

The world model represents the model of the world from the perspective of an agent. Further, the world model can be an interface between the ALICA engine and other software, e.g., ROS and lower API.

![coreconcept](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/coreconcept.png)

### 4. Setup of the Catkin Workspace
We need to creating a catkin workspace by executing the following steps in an Ubuntu18.04 terminal.

1. Check out the required repositories:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rapyuta-robotics/alica.git
git clone https://github.com/rapyuta-robotics/alica-essentials.git
git clone https://github.com/rapyuta-robotics/alica-supplementary.git
```

2. Remove existing turtlesim files. You will reproduce these files (You can jump to step 8. "Build and Run" for testing application before deleting the files):

```
cd src/alica-supplementary
rm -r alica_ros_turtlesim/Expr 
rm -r alica_ros_turtlesim/alica
```

3. Create the alica_ros_turtlesim package with additional subfolders:

```
catkin_create_pkg alica_ros_turtlesim roscpp turtlesim geometry_msgs alica_engine agent_id alica_ros_proxy constraintsolver
mkdir -p alica_ros_turtlesim/alica/etc/plans/behaviours
mkdir -p alica_ros_turtlesim/alica/etc/roles
mkdir -p alica_ros_turtlesim/alica/etc/Misc
mkdir -p alica_ros_turtlesim/Expr
```

### 5. Setup of the ALICA Plan Designer
The ALICA plan designer is a user interface to design applications with the ALICA framwork.
##### Start alica plan designer by following steps

```
cd catkin_ws/src/alica-plan-designer
./bin/start.sh
```
![plan-designer_project_config](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/plan-designer_project_config.png)
##### Plan designer setting
Create project: input root path to your alica ws and change expr as well.
#todo add image
Window -> Preference -> Plan Designer -> Condition plugin
/home/yu/alica_ws/src/alica-plan-designer/ should be path to alica-plan-designer
![plan-designer_condition_plugin](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/plan-designer_condition_plugin.png)

### 6. Create plan using plan desginer
 In this section, you will create plan using plan designer. This tutorial has two plans, `Master` and `Move`. `Master` plan has `Init` and `Move` state. `Move` plan has `Move2Center` and `AlignCircle` state.
 In the `Init` state, turtle is teleported to the random state(`Go2RandomPosition` behaviour) and transit to the `Move` state. 
![plan](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/plan.png)


##### 6.1 Plan design		
###### 6.1.1 Master plan
![master_plan](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/master_plan.png)
1. right click on Explore -> New -> Plan
2. input Filename with `Master` then plan will open.
3. click top blue `Master` and change `Master Plan` true in the Properties.(top bar become red from blue)
4. Change “NewState“ with `Init`
5. Add state named `Move`
6. Add transition `Init` to `Move` and `Move` to `Init`.
7. Add `Go2RandomPosition` bahaviour to “Init” state under /Plans/behaviours
8. Add `Move` plan to `Move` state under /Plans

###### 6.1.2 Move plan
![move_plan](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/move_plan.png)
1. Add `Move2Center` state and `AlignCircle` state
2. Add `GoTo` behaivour to  `Move2Center` state and `AlignCircle` state under /Plans/behaviours
3. Add “Entry point” and Task.
Change Task name to `Leader` and `Follower`.
4. Add “Runtime Condition” to the plan.
5. Add “x, y” “Quantifiers” in Runtime condition properties
				* these quantifiers will be used in writing constraints.
![runtime_condition](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/runtime_condition.png)

###### 6.1.3 Add Role
1. Define Role -> “Add Role” -> input “Turtle”
** Role is physical difference, i.e. Arm and AGV 
2. Right click on Explore -> New -> Roleset -> select “Master.pml” in Choose plan 
3. Open Roleset.rset -> Expand all -> auto layout, then you will see
#todo add iamge
Change Default to true as above image.
![roleset](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/roleset.png)


4. Generate code by open any plan -> right click -> Codegeneration -> Generate all expression validators

### 7. Implement Logics 
After ALICA generate code structure, developer need to modify/implement logic. In this section, necessary code changes are explained. 
#### 7.1 World model
We will explain only `base_node.cpp` which is related to ALICA. 
![base](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/base.png)
- L13: initialize World model. Since one agent has one world model, this method is class static method
- L15~19: Setting up ALICA-supplementray. ALICA supplementary is supporting class of ALICA.
- L21~31: Setting up ALICA Engine. 

#### 7.2 Plan
<plan name>.cpp file has "evaluate" function runs with specific frequency which is set in Alica.conf. "evaluate" function has state transition logic. constraints/<plan name>.cpp has "getConstraints" function which is used to provide constraints.

- Expr/src/Plans/Master***.cpp(*** is a id number which is generated by alica-plan-designer)
    In this file, you need to implement state transition logic.
    ![master](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/master.png)
    - L3 : include world model since state transition happen depends on world model

    - L47:  This is state transition condition from ‘Init’ to ‘Move’. This success condition is met by setSuccess in behaviour logic which is explained later in behaviour.
    - L75: This is state transition condition from ‘Move’ to ‘Init’. This condition is met by publishing rostopic ‘/init’.
	
- Expr/src/Plans/constraints/Move***.cpp
	In this file, you need to implement constraints logic.
    ![constraints](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/constraints.png)

    - L5~6: include alica engine related header files and world model.    
    - L48~106: Iterate all agents and add constraints
    - L50~70: Preparing/getting variables and add range constraints
    - L74: add constraints for `Leader` agent. Leader agent moves to center of circle within tolearence
    - L83~L99: Iterate other agent to add constraints to keep distance among `Follower` agents.

#### 7.3  Behaviour
<behaviour name>.cpp file has `run` function which keep running at specific frequency which is set in Alica.conf.
- Expr/src/Behaviours/Go2RandomPosition.cpp
    In this file, implement turtle teleportation
	![go2randomposition](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/go2randomposition.png)

    - L6: include world model
	- L20-29: generate random value and teleport turtle via world model.
    - L30: After execute `setSuccess()` function, "isAnyChildStatus(PlanStatus::Success)"" at Expr/src/Plans/Master***.cpp return true. Then state transition from `Init` to `Move` happen.

- Expr/src/Behaviours/GoTo.hpp

    ![gotohpp](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/gotohpp.png)
    - L6: include Qurty
    - L24-25: add query and result which are used to get solver result.

- Expr/src/Behaviours/GoTo.cpp
  
    ![gotocpp](https://github.com/rapyuta-robotics/alica-supplementary/raw/rr-devel/alica_ros_turtlesim/doc/gotocpp.png)
    - L5: include world model
    - L27~30: In `run` function,  get result from solver. CGSolver solve constraints defined in the  Expr/src/Plans/constraints/Move***.cpp
    - L32~33: move turtle to the solver result position via world model.
    - L41~43: add variable to query. “x” and “y” was defined when  “Quantifiers” of “Runtime Condition” was added in the alica-plan-designer

### 8. Build and Run
#### 8.1 Build
follow the standard ros build step.
```
cd catkin_ws
source /opt/ros/kinetic/setup.bash
catkin build alica_ros_turtlesim
```
#### 8.2 Run
Run application with roslaunch. video
- Launch turtlesim
			`roslaunch alica_ros_turtlesim env.launch`
- Turtle node(you can launch turtle with different name till 5 turtles)
`roslaunch alica_ros_turtlesim turtle.launch name:=turtle1`
- Start moving. 
`rostopic pub /init std_msgs/Empty "{}" `

### 9. ADVANCE Tutorial(Coming Soon?)
- Debugging tools
- App with constraints utility
    modify constraints
- App with inter agent communication
    implement inter agent communication
    modify constraints
- App with Utility function
    modify with plan designer
    implement utility function
    each turtle have different spec, e.g. speed, size and etc
    customize utility function

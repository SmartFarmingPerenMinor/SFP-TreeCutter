
# Testplan period 1

## Introduction
This is a testplan written for the real UR10e

## To be tested
- Rviz visualizes the UR10e
- Moving the UR10e using RViz
- Moving the UR10e using our code

## Not to be tested
- Smooth planning
- Settings for movement

## Testing
BEFORE WE BEGIN, ALWAYS, ALWAYS have someone ready to press the EMERGENCY STOP button.

### Installation of steps
Follow the main repository's README to install the environment.
We use the ur_robot_driver package from [Universal_Robots](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver).

And our testing package [test](https://github.com/SmartFarmingPerenMinor/SFP-TreeCutter/tree/master/catkin_ws/src/test)

ALWAYS source catkin_ws/devel/setup.bash whenever you open a new terminal.

#### Starting up

To set up the program, move to dir SFP-TreeCutter:
(To get the connection and calibration working, refer to [start connection](https://github.com/SmartFarmingPerenMinor/SFP-TreeCutter/tree/master/Documentation/start_connection.md))

Open terminal:
```bash
source catkin_ws/devel/setup.bash
./start_connection.sh <robot_ip_address> 

```
<robot_ip_address> is optional. It has a default value set.
It is not recommended to run the ./start_connection.sh in parallel, so as to end the process with CTRL + C / CTRL + Z

Once connected, 
start two new terminals and run:
```bash
source catkin_ws/devel/setup.bash
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch
```

```bash
source catkin_ws/devel/setup.bash
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true limited:=true
```

If you want to run it in parallel, add an & to the command like so.
```bash
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch &
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true limited:=true &
```
This should boot up Rviz. Once it has booted up, add the motion planner.
![add_robot](./Images/add_robot.png)
Attach the base_link fixed frame to set the orientation and planner space.
![fix_frame](./Images/fix_frame.png)

You should see the arm represented in Rviz and if all has been set up correctly, it should be reflective of the arm's current actual state.

### Moving the arm using Rviz

Set the planning group from end_effector to manipulator.
![change_planning](./Images/change_planning.png)
This gives access to a transform bauble, which can be used to manipulate the planner.
![show_bauble](./Images/show_bauble.png)
Press Plan & Execute to move the arm towards the planner.
If it fails, it may mean the joint limits have been exceeded.
![planner_limits](./Images/planner_limits.png)
Always check if the connection is still active.

### Moving the arm using code

Make sure your terminal has been sourced (otherwise run source catkin_ws/devel/setup.bash).
Run 

```bash
rosrun test main.py
```
Then, input waypoints as you wish. The coordinate space is relative to the end-effector, in space and orientation as shown in the teach pendant.
Waypoints are set in meters.
(X = Left -, Right +)
(Y = Back -, Forth +)
(Z = Down -, Up +)


## Criteria for succes
#### Download criteria
- Can you download the files?

#### setup the file criteria
- Have you been able to build (or make) the project?

#### Start criteria
- Has the connection been set properly?
- Does Rviz correctly reflect the arm's current state?

#### End criteria
- Can you move the planner around?
- Does the arm move to the planned point?


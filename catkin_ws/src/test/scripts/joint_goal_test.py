#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def main():
    # init commander / rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_move", anonymous=True)

    # instantiate the robot
    robot =  moveit_commander.RobotCommander()

    # instantiate the scene
    scene = moveit_commander.PlanningSceneInterface()

    # instantiate moveGroupCommander
    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size = 20,
    )

    planning_frame = move_group.get_planning_frame()
    print("=== planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("=== end effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("=== available planning groups:", robot.get_group_names())

    print("=== robot state")
    print(robot.get_current_state())
    print("")

    #We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 4
    joint_goal[2] = -tau / 8
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


def plan_cartesian_path(move_group, scale=1):
    waypoints = []
    
    start_pose = move_group.get_current_pose().pose
    waypoints.append(copy.deepcopy(start_pose))

    second_pose = start_pose
    

    waypoints.append(copy.deepcopy(second_pose))

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.1, 0.0)

    print("plan: ", plan)
    print("fraction: ", fraction)

    return plan, fraction

def execute_plan(move_group, plan):
    move_group.execute(plan, wait=True)

if __name__ == "__main__":
    main()


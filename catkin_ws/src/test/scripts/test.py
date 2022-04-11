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

    move_group.set_named_target('home')

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0

    received_x = float(input('Enter a x coordinate: '))
    received_y = float(input('Enter a y coordinate: '))
    received_z = float(input('Enter a z coordinate: '))

    pose_goal.position.x = received_x
    pose_goal.position.y = received_y
    pose_goal.position.z = received_z
    
    move_group.set_pose_target(pose_goal)

    #plan the path
    plan = move_group.plan()
    plan_succes = plan[0]

    #check if the resulting path is valid and exit if not
    if plan_succes == False:
        print("planning failed, exiting")
        return

    print("planning succeded, moving")
    plan = move_group.go(wait=True)

    move_group.stop()

    #move_group.clear_pose_targets()


    #cartesian_plan, fraction = plan_cartesian_path(move_group)
    
    #move_group.execute(cartesian_plan)


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


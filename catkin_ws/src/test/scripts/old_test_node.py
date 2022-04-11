#!/usr/bin/env python3.8

from copy import deepcopy
from socket import MSG_NOSIGNAL

import sys
import rospy
import moveit_commander
from moveit_commander import *
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion

def main():    
# Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the ROS node
    rospy.init_node('ur10_e_move_test_old', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Connect to the manip move group
    move_group = MoveGroupCommander('manipulator')

    # Allow replanning to increase the odds of a solution
    move_group.allow_replanning(True)

    # Set the reference frame
    move_group.set_pose_reference_frame('base_link')
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    # Allow some leeway in position(meters) and orientation (radians)
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.1)

    # Get the name of the end-effector link
    end_effector_link = move_group.get_end_effector_link()

    # Start in the "straight_forward" configuration stored in the SRDF file
    move_group.set_named_target('home')

    # Plan and execute a trajectory to the goal configuration
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # Get the current pose so we can add it as a waypoint
    start_pose = move_group.get_current_pose(end_effector_link).pose
    start_pose.orientation.w = 1.0

    # Initialize the waypoints list
    waypoints = []

    # Set the first waypoint to be the starting pose
    # Append the pose to the waypoints list
    waypoints.append(start_pose)

    wpose = deepcopy(start_pose)

    # Set the next waypoint back 0.2 meters and right 0.2 meters
    wpose.position.z -= 0.2  # First move up (z)
    wpose.position.y += 0.2  # and sideways (y)

    # Append the pose to the waypoints list
    waypoints.append(deepcopy(wpose))


    # Set the next waypoint to the right 0.15 meters
    wpose.position.x += 0.1  # Second move forward/backwards in (x)

    # Append the pose to the waypoints list
    waypoints.append(deepcopy(wpose))

    wpose.position.y -= 0.2  # Third move sideways (y)

    # Append the pose to the waypoints list
    waypoints.append(deepcopy(wpose))

    fraction = 0.0
    maxtries = 100
    attempts = 0

    # Set the internal state to the current state
    move_group.set_start_state_to_current_state()

    # Plan the Cartesian path connecting the waypoints
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = move_group.compute_cartesian_path (
        waypoints, # waypoint poses
        0.01, # eef_step
        0.0, # jump_threshold
        True) # avoid_collisions

        # Increment the number of attempts
        attempts += 1

        # Print out a progress message
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts)
            + " attempts...")

        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")

        move_group.execute(plan)
        move_group.stop()
        move_group.clear_pose_targets()

        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " +
        str(fraction) + " success after " + str(maxtries) + " attempts.")

        # Move normally back to the 'resting' position
        # move_group.set_named_target('up')
        # move_group.go()
        # rospy.sleep(1)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

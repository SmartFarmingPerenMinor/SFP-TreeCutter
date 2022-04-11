#!/usr/bin/env python3.8

from copy import deepcopy
from gc import get_objects
from io import StringIO
from socket import MSG_NOSIGNAL

import sys
from dataclasses import dataclass
from typing import Any, List, Tuple
import rospy
from rospy import ServiceProxy
import moveit_commander
from gazebo_msgs.srv import *
from moveit_commander import *
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from geometry_msgs.msg import Pose, Point, Quaternion
import readline


class GazeboModel:

    def __init__(self,  name: str, relative_entity_name: str):
        self.name = name
        self.relative_entity_name = relative_entity_name

class MoveGroupObj:

    def __init__(self, move_group_name: str):
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group: MoveGroupCommander = MoveGroupCommander(move_group_name)
        self.robot: RobotCommander = RobotCommander()
        self.scene: PlanningSceneInterface = PlanningSceneInterface()
        self.ps: PlanningScene = PlanningScene()

def main():    

    move_group = init("manipulator")
    # plant_tree()
    # show_gazebo_models()
    # get_gazebo_models()
    waypoints, max_tries, allowed_fraction = set_waypoints(move_group)
    print(f"waypoints:\n{waypoints},\n max_tries: {max_tries},\n allowed_fraction: {allowed_fraction}\n")
    
    cartesian_path_execution(move_group, waypoints, max_tries, allowed_fraction)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

def get_gazebo_models():
    # try:
    #     world_properties = ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    #     resp1 = world_properties()
    #     rospy.loginfo(resp1)
    # except rospy.ServiceException as e:
    #         rospy.loginfo("Get World Properties service call failed:  {0}".format(e))
    success = True

    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        resp1 = get_world_properties()
    except rospy.ServiceException:
        success = False
    if success:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        for model in resp1.model_names:
            try:
                model_properties = get_model_properties(model)
                print(model_properties)
            except rospy.ServiceException:
                success = False
    return resp1        

def show_gazebo_model_states():
        try:
            model_coordinates = ServiceProxy('/gazebo/get_model_state', GetModelState)
            tree_name = "my_mesh"
            tree = GazeboModel(tree_name, "body")
            
            resp_coordinates = model_coordinates(tree.name, tree.relative_entity_name)

            print('\n')
            print('Status.success = '+ str(resp_coordinates.success))
            print(tree_name)
            print("Value X : " + str(resp_coordinates.pose.position.x))
            print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

def plant_tree():

    rospy.sleep(2)
    tree_name = "my_mesh"
    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=30)
    scene.remove_world_object(tree_name)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 2.0
    p.pose.position.y = 2.0
    p.pose.position.z = 0.
    scene.add_mesh(tree_name, p, "/home/anthony/Desktop/SFP/SFP-SimulationFresh/catkin_ws/src/test/worlds/../models/peren.dae")
    scene_pub.publish(PlanningScene)
    rospy.loginfo("Adding "+ tree_name)
    rospy.sleep(2)
    # Connect to the manip move group

def init(move_group_name: str) -> MoveGroupObj:
    # Initialize the ROS node
    rospy.init_node('ur10_e_move_test')
    move_grouper = MoveGroupObj(move_group_name)

    move_group: MoveGroupCommander = move_grouper.move_group
    robot: RobotCommander = move_grouper.robot

    # Allow replanning to increase the odds of a solution
    move_group.allow_replanning(True)
    move_group.allow_looking(True)

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
    print("============ Available Planning Groups:", group_names)
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    # Allow some leeway in position(meters) and orientation (radians)
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.1)
    move_group.set_num_planning_attempts(10)

    # Start in the "straight_forward" configuration stored in the SRDF file
    move_group.set_named_target('home')

    # Plan and execute a trajectory to the goal configuration
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    move_group.set_goal_tolerance

    return move_grouper

def set_waypoints(move_group: MoveGroupCommander) -> Tuple[list, int, float]:
    
    max_tries: int = 30
    allowed_fraction: float = 0.95 # Between 0 and 1

    # Get the name of the end-effector link
    end_effector_link = move_group.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
    start_pose = move_group.get_current_pose(end_effector_link).pose
    start_pose.orientation.w = 1.0

    # Initialize the waypoints list
    waypoints = []

    # Set the first waypoint to be the starting pose
    # Append the pose to the waypoints list
    waypoints.append(start_pose)
    wpose = deepcopy(start_pose)

    add_more_waypoints: bool = True

    while(add_more_waypoints):

        try:
            rx, ry, rz = input('Enter x, y, z coordinates \n[example 0.1, 0, 0.2]: ').split(",")
            
            wpose.position.x += float(rx)
            wpose.position.y += float(ry)
            wpose.position.z += float(rz)
            waypoints.append(deepcopy(wpose))

            exit_str:str = input('More coordinates? [Type yes for more]: ').lower()

            if exit_str == "yes":
                continue
            else:
                allowed_fraction = float(input(f"Enter allowed fraction [Default {allowed_fraction} for {allowed_fraction * 100}%]: ") or str(allowed_fraction))
                max_tries: int = int(input(f"Enter maximum attempts [Default {max_tries}]: ") or str(max_tries))
                add_more_waypoints = False

        except ValueError:
            print("Invalid input")
            
        else:
            break
    
    return (waypoints, max_tries, allowed_fraction)

def cartesian_path_execution(move_group: MoveGroupCommander, waypoints: list, max_tries: int, allowed_fraction: int):

    fraction: float = 0.0
    attempts: int = 0
    # current_time: int
    # start_time: int = rospy.get_time()


        # Plan the Cartesian path connecting the waypoints
    while fraction < 1.0 and attempts < max_tries:
        (plan, fraction) = move_group.compute_cartesian_path (
        waypoints, # waypoint poses
        0.01, # eef_step
        0.0, # jump_threshold
        True) # avoid_collisions
        # Increment the number of attempts
        attempts += 1
        # current_time = rospy.get_time() - start_time
        # print("Time elapsed: "+ current_time)
        # for joint_pos_v in move_group.get_current_joint_values():
        #     print("Joint position: " + joint_pos_v)

        # Print out a progress message
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts)
            + " attempts...")

        # If we have a complete plan, execute the trajectory
        if fraction > allowed_fraction:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            move_group.execute(plan, wait=True)
            rospy.sleep(1)
            move_group.stop()
            move_group.clear_pose_targets()

            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with " +
            str(fraction) + " success after " + str(attempts) + " attempts.")


if __name__ == "__main__":
    main()

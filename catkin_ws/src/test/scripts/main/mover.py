#!/usr/bin/python3

from copy import deepcopy
from typing import Tuple
import rospy
import moveit_commander
#For vsc.. intellisense
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory, PositionConstraint, OrientationConstraint, Constraints
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import ColorRGBA
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

#For arrow-keys in cmd
import readline


from math import pi, tau, dist, fabs, cos, sin

from moveit_commander.conversions import pose_to_list

class endEffectorMover:
    marker_id_counter = 0
    def __init__(self, arguments):
        # init commander / rospy node

        moveit_commander.roscpp_initialize(arguments)
        
        # instantiate the robot
        self.robot = RobotCommander()

        # instantiate the scene
        self.scene = PlanningSceneInterface()

        # instantiate moveGroupCommander
        group_name = 'manipulator'
        self.move_group = MoveGroupCommander(group_name)
        self.waypoints = []
        self.max_tries: int = 30
        self.allowed_fraction: float = 0.95 # Between 0 and 1
            # Allow replanning to increase the odds of a solution

        # self.move_group.set_planning_pipeline_id("OMPLPlanner")
        # self.move_group.set_planner_id("RRTConnect")

        self.move_group.allow_replanning(True)
        # self.move_group.allow_looking(True)
                
        # Allow some leeway in position(meters) and orientation (radians)
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.1)
        self.move_group.set_num_planning_attempts(200)
        self.move_group.set_planning_time(10)
        rospy.on_shutdown(self.move_group.stop)
        rospy.on_shutdown(self.move_group.clear_pose_targets)
        rospy.on_shutdown(mover_exit)
        # subscribe to the topic
        self.display_trajectory_publisher = rospy.Publisher(
                "/move_group/monitored_planning_scene",
                DisplayTrajectory,
                queue_size = 20,
        )

        self.marker_publisher = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=20,
        )

    def printInfo(self):

        planning_frame = self.move_group.get_planning_frame()
        print("=== planning frame: %s" % planning_frame)

        planning_pipeline = self.move_group.get_planning_pipeline_id()
        print("=== planning pipeline: %s" % planning_pipeline)
 
        eef_link = self.move_group.get_end_effector_link()
        print("=== end effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("=== available planning groups:", group_names)

        robot_state = self.robot.get_current_state()
        print("=== robot state")
        print(robot_state)

        
        print("")

    def prompt_location(self):
        prompt: bool = True
        try:
            button = input("[Enter] for coord, [H] for home: ")
        except ValueError:
            print("Invalid value. Enter a valid value.")
        if button == "":
            while(prompt):
                try:
                    x = float(input('Enter a x coordinate: '))
                    y = float(input('Enter a y coordinate: '))
                    z = float(input('Enter a z coordinate: '))
                except ValueError:
                    print("Invalid value. Enter a valid value.")
                else:
                    prompt = False

            self.move_to(x,y,z)
        elif button.lower() == "h":
            self.move_group.set_named_target("home")
            plan = self.move_group.plan()
            self.move(plan[0], plan[1])
            self.prompt_location()

    def move_to(self, x, y, z):
        """Move to quaternion space pose goal"""
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group = self.move_group

        move_group.set_pose_target(pose_goal)
        plan = self.move_group.plan()

        self.move(plan[0], plan[1])

        if not prompt_continue("[Enter] continue, or [X] shutdown: "):
            rospy.signal_shutdown("Exit")
        self.prompt_location()
        

    def visualize_planning(self, plan) -> bool:
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def set_waypoints(self):

        # Get the name of the end-effector link
        end_effector_link = self.move_group.get_end_effector_link()

        # Get the current pose so we can add it as a waypoint
        start_pose = self.move_group.get_current_pose(end_effector_link).pose
        start_pose.orientation.w = 1.0

        # Initialize the waypoints list
        self.waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list << For some reason this breaks the program.
        # waypoints.append(start_pose) # << Try it if you wish.

        wpose = deepcopy(start_pose)

        add_more_waypoints: bool = True

        while(add_more_waypoints):

            try:
                rx, ry, rz = input('Enter x, y, z coordinates \n[example 0.1, 0, 0.2]: ').split(",")
                
                wpose.position.x += float(rx)
                wpose.position.y += float(ry)
                wpose.position.z += float(rz)
                self.waypoints.append(deepcopy(wpose))

                if not prompt_continue("[Enter] next phase, or [X] more waypoints: "):
                    continue

                else:
                    add_more_waypoints = False
                    if not prompt_continue("[Enter] next phase, or [X] change settings: "):
                        self.allowed_fraction = float(input(f"Enter allowed fraction [Default {self.allowed_fraction} for {self.allowed_fraction * 100}%]: ") or str(self.allowed_fraction))
                        self.max_tries: int = int(input(f"Enter maximum attempts [Default {self.max_tries}]: ") or str(self.max_tries))

            except ValueError:
                print("Invalid input")
        

    def cartesian_path_execution(self):

        fraction: float = 0.0
        attempts: int = 0

        self.set_waypoints()

        if not prompt_continue("Waypoints planned. [Enter] to execute, [X] to abort."):
            rospy.signal_shutdown("Exit")

        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < self.max_tries:
            (plan, fraction) = self.move_group.compute_cartesian_path (
            self.waypoints, # waypoint poses
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
            if fraction > self.allowed_fraction:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                # plan_success = True because fraction already determines plan completion
                self.move(True, plan)

                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with " +
                str(round(fraction, 2)) + " success after " + str(attempts) + " attempts.")

    def moveToTree(self, point, printPoint=False):
        if (printPoint):
            print (point)
        self.move_to(point["x"], point["y"], point["z"])

    def move(self, plan_success: bool, trajectory: RobotTrajectory):
        """Plan and execute as set up"""
        self.move_group.construct_motion_plan_request()
        # Plan the path, note: planning requires goal to be set.

        #check if the resulting path is valid and exit if not
        if plan_success == False:
            print("Planning failed!")
            if not prompt_continue("[Enter] reattempt, or [X] shutdown: "):
                rospy.signal_shutdown("Exit")
            
        # self.visualizePlanning(plan)
        print("Executing planning: moving")

        # plan[1] = RobotTrajectory
        self.move_group.execute(trajectory, wait=True)
        rospy.sleep(1)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def calcQuaternions(self, phi, theta, psi):
        qw = cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)
        qx = sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)
        qy = cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)
        qz = cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)
        return qw, qx, qy, qz


def prompt_continue(prompt_text: str):
    try:
        char = input(prompt_text)
    except ValueError:
        print("Invalid value. Enter a valid value.")
    if char.lower() == "x":
        return False
    elif char == "":
        return True    

def mover_exit():
    print("Program exited. Goodbye.")
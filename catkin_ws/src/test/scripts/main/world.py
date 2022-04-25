#!/usr/bin/python3

import rospy
import moveit_msgs.msg
import geometry_msgs.msg
#For vsc.. intellisense
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory, PositionConstraint, OrientationConstraint, Constraints
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import ColorRGBA
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from mover import endEffectorMover

class worldBuilder:

    def __init__(self, emo: endEffectorMover) -> None:
        self.emo = emo

    def addPlane(self, planeName :  str, x : float = 0, y: float = 0, z: float = 0, normX: float = 0, normY: float = 0, normZ: float = 0):
        rospy.sleep(1)
        plane_pose = geometry_msgs.msg.PoseStamped()
        plane_pose.header.frame_id = self.emo.move_group.get_planning_frame()
        plane_pose.pose.orientation.w, plane_pose.pose.position.x, plane_pose.pose.position.y, plane_pose.pose.position.z = self.emo.calcQuaternions(x, y, z)
        self.emo.scene.add_plane(planeName, plane_pose, normal=(normX,normY,normZ))
        print(" added " , planeName, "to ", self.emo.__class__.__name__, " perception")

    def create_simple_box_constraints(self):
        pcm = PositionConstraint()
        pcm.header.frame_id = self.emo.move_group.get_pose_reference_frame()
        pcm.link_name = self.emo.move_group.get_end_effector_link()

        cbox = SolidPrimitive()
        cbox.type = SolidPrimitive.BOX
        cbox.dimensions = [3, 0.2, 3]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.emo.move_group.get_current_pose()

        cbox_pose = Pose()
        cbox_pose.position.x = current_pose.pose.position.x - 0.05
        cbox_pose.position.y = -0.3
        cbox_pose.position.z = 0
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm
        
    def display_box(self, pose, dimensions):
        """ Utility function to visualize position constraints. """
        assert len(dimensions) == 3

        # setup cube / box marker type
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.emo.marker_id_counter
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.color = ColorRGBA(0.0, 0.0, 0.0, 0.5)
        marker.header.frame_id = self.emo.move_group.get_pose_reference_frame()

        # fill in user input
        marker.pose = pose
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]

        # publish it!
        self.emo.marker_publisher.publish(marker)
        self.emo.marker_id_counter += 1

    def test_constraints(self):
        pcm = self.create_simple_box_constraints()
        path_constraints = Constraints()
        path_constraints.position_constraints.append(pcm)
    
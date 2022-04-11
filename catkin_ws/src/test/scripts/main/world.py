#!/usr/bin/python3

import rospy
import moveit_msgs.msg
import geometry_msgs.msg

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

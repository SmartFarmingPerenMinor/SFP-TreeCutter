#!/usr/bin/python3

import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer

def main():
    endEffectorMoverObject = endEffectorMover(sys.argv)
    # depthViewerObject = depthViewer()
    endEffectorMoverObject.promptLocationAndMove()
    # endEffectorMoverObject.cartesian_path_execution()
    
if __name__ == "__main__":
    try: 
        while not rospy.is_shutdown():
            rospy.init_node("main_process_node")
            main()
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted!")

#!/usr/bin/python3

import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder

def main():

    rospy.init_node("main_py", disable_signals=True)
    endEffectorMoverObject = endEffectorMover(sys.argv)
    worldBuilderObject = worldBuilder(endEffectorMoverObject)
    worldBuilderObject.test_constraints()
    worldBuilderObject.addPlane("ground_plane")
    # depthViewerObject = depthViewer()
    # endEffectorMoverObject.prompt_location()
    # endEffectorMoverObject.cartesian_path_execution()
    
if __name__ == "__main__":
    try: 
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted!")

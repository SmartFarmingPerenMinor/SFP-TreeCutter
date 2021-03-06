#!/usr/bin/python3

import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder
from benchmark import Benchmark

def main():

    rospy.init_node("move_group_python", disable_signals=True)
    endEffectorMoverObject = endEffectorMover(sys.argv)
    
    # depthViewerObject = depthViewer()
    # endEffectorMoverObject.prompt_location()
    endEffectorMoverObject.cartesian_path_execution()
    #bmark = Benchmark(endEffectorMoverObject)
    #bmark.vLine()

if __name__ == "__main__":
    try: 
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted!")

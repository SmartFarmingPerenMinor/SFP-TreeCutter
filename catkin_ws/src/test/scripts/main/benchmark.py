#!/usr/bin/python3

import time
import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder

def main():

    rospy.init_node("move_group_python", disable_signals=True)
    endEffectorMoverObject = endEffectorMover(sys.argv)
    # move to home position
    #endEffectorMoverObject.move_to(0.7, 0.7, 0.6, False)
    #move to predefined point
    #endEffectorMoverObject.move_to(0.7, 0.7, 0.6, False)

if __name__ == "__main__":
    benchmarks = []
    for i in 10
    try: 
        if not rospy.is_shutdown():
                #startTime = time.time()
                main()
                #endTime = time.time()
                #benchmark = {"timeID":i + 1, "time":(endTime-startTime)}
                #benchmarks.append(benchmark)

    except rospy.ROSInterruptException:
        rospy.loginfo("Process interrupted!")

    print("\n\nprinting benchmarks results: ")
    print(benchmarks)

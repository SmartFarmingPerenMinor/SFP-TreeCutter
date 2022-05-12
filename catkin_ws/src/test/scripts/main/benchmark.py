#!/usr/bin/python3

import time
from datetime import datetime, timedelta
import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder
"""
  position: 
    x: 0.12432091289173712
    y: 0.5199326845855795
    z: 0.5404726440432062
  orientation: 
    x: 0.5643447768776021
    y: 0.7249470756015347
    z: 0.1987028229978667
    w: 0.34129737549596056
"""

"""  
  position: 
    x: 0.15053777838975663
    y: 0.42005508485776083
    z: 1.1709618143256848
  orientation: 
    x: -0.6770049105444241
    y: -0.5902159792009894
    z: -0.4376494503671559
    w: 0.04209997135248066
"""



def main(endEffectorMoverObject: endEffectorMover):
    # print(endEffectorMoverObject.move_group.get_current_pose(endEffectorMoverObject.move_group.get_end_effector_link()))
    
    # move to home position
    endEffectorMoverObject.set_waypoints_manual(0.12432091289173712, 0.5199326845855795, 0.5404726440432062, 0.34129737549596056)
    #move to predefined point
    endEffectorMoverObject.set_waypoints_manual(0.15053777838975663, 0.42005508485776083, 1.1709618143256848, 0.04209997135248066)
    
    
if __name__ == "__main__":
    benchmarks = []
    rospy.init_node("benchmark", disable_signals=True)
    endEffectorMoverObject = endEffectorMover(sys.argv)
    for i in range(10):
        try: 
            if not rospy.is_shutdown():
                    startTime = time.time()
                    main(endEffectorMoverObject)
                    endTime = time.time()
                    benchmark = {"timeID":i + 1, "time":(endTime-startTime)}
                    benchmarks.append(benchmark)

        except rospy.ROSInterruptException:
            rospy.loginfo("Process interrupted!")

    print("\n\nprinting benchmarks results: ")
    print(benchmarks)
    
    fd = open("benchmarkResult.txt", 'a')
    for i in range(0, len(benchmarks)):
        logMsg = f"data {datetime.today().strftime('%Y-%m-%d')} {timedelta(seconds=666)}: {benchmarks[i]}\n"
        fd.write(logMsg)
    fd.close()

#!/usr/bin/python3

import time
from datetime import datetime, timedelta
import sys
import rospy

from mover import endEffectorMover
from camera import cameraViewer
from depth_camera import depthViewer
from world import worldBuilder

def main(endEffectorMoverObject: endEffectorMover):
    # move to home position
    endEffectorMoverObject.move_to(-0.7094929255872758, 0.6859538724164436, 0.8130753580446348, False)
    #move to predefined point
    endEffectorMoverObject.move_to(0.2846635174250878, 0.7970207115848729, 0.5420001587584037, False)

if __name__ == "__main__":
    benchmarks = []
    rospy.init_node("move_group_python", disable_signals=True)
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

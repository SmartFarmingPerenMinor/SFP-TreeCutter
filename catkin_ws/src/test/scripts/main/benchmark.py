#!/usr/bin/python3

import time
from datetime import datetime, timedelta
import sys

from mover import endEffectorMover

class Benchmark()
    def __init__(self, endeffector: endEffectorMover, logFile: str = "benchmarkResult.txt"):
       self.endeffector = endeffector
       self.logFile = logFile

    def vLine(self, numOfTimes: int = 10):
        benchmarks = []
        for i in range(numOfTimes):
            startTime = time.time()
            # move to home position
            self.endeffector.set_waypoints_manual(0.12432091289173712, 0.5199326845855795, 0.5404726440432062, 0.34129737549596056)
            #move to predefined point
            self.endeffector.set_waypoints_manual(0.15053777838975663, 0.42005508485776083, 1.1709618143256848, 0.04209997135248066)
            endTime = time.time()
            benchmark={"type": "line benchmark", "ID": i+1,"time":(endTime-startTime)}
            benchmarks.append(benchmark)
        self.__logToFile__(benchmarks)


    def __logToFile__(self, benchmarks):
        fd = open(self.logFile, 'a')
        for i in range(0, len(benchmarks)):
            logMsg = f"data {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: {benchmarks[i]}\n"
            fd.write(logMsg)
        fd.close()

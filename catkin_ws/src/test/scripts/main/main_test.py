#!/usr/bin/python3

import sys

from mover import endEffectorMover
from camera import cameraViewer
from pointCloud2 import PointCloud

def main():
    endEffectorMoverObject = endEffectorMover(sys.argv)
    cameraViewerObject = cameraViewer()

    pointCloud = PointCloud()

    endEffectorMoverObject.moveToTree(pointCloud.getRandomPoint(), True)

if __name__ == "__main__":
    main()

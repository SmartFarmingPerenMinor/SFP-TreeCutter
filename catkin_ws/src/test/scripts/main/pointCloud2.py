#!/usr/bin/python3

import rospy, random
from ros_numpy import point_cloud2 as nmPlc
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class PointCloud():
    points = []

    def __init__(self, pointCloud2_topic = "/ur10e/camera1/depth/points"):
        random.seed(rospy.Time.now())
        rospy.Subscriber(pointCloud2_topic, PointCloud2, self.__callback, queue_size=1)

    def __callback(self, msg):
        self.points = []
        res = point_cloud2.read_points(msg, field_names= ("x", "y", "z"), skip_nans=True)
        for p in res:
            point = {"x": p[0], "y": p[1], "z": p[2]}
            self.points.append(point)
        self.getRandomPoint()

    def getRandomPoint(self):
        point = self.points[random.randrange(0, len(self.points), 1)]
        return point

    def sleep(self):
        rospy.sleep(0.8)

    def shutdown(self):
        self.__del__()

    def __del__(self):
        nodeName = rospy.get_name()
        print (f"stoping ros node: {nodeName}.")
        exit_msg = f"programme ended, stopping ros node: {nodeName}."
        rospy.signal_shutdown(exit_msg)

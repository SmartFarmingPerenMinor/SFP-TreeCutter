#!/usr/bin/env python3

from cmath import isnan
import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class depthViewer:
    def __init__(self, topic: str = "/ur10e/camera1/depth/image_raw"):
        self.topic = topic
        self.bridge = CvBridge()
        self.image = None
        self.sub = rospy.Subscriber(topic, Image, self.image_callback)
        rospy.on_shutdown(cv2.destroyAllWindows)

    def image_callback(self, msg: Image):
        rospy.loginfo("Establishing depth img Callback")
        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.image = image
        self.depth_display(msg)


    def depth_display(self, msg: Image):
        image_msg = self.bridge.cv2_to_imgmsg(self.image)
        # print(f"Image: {image_msg} ")      
        # mask = np.zeros(self.image.shape,np.uint8)
        # mask = mask[~np.isnan(self.image)]
        # pixelpoints = cv2.findNonZero(mask)
        # print(pixelpoints)
        # rows,cols = self.image.shape
        # for i in range(rows):
        #     for j in range(cols):
        #         k = self.image[i,j]
        #         if (not np.isnan(k)):
        #             print(k)
        
        # min, max, minLoc, maxLoc = cv2.minMaxLoc(self.image)
        # x, y, w, h = self.crop_fastest(self.image)
        cv2.imshow('Depth view', self.image)
        cv2.waitKey(2)
        # print(self.image)
        # print("Displaying depth: \nmin:", min, "\nmax:", max, "\nminLoc:", minLoc, "\nmaxLoc:", maxLoc)

    def crop_fastest(self, arr):
        return cv2.boundingRect(arr)

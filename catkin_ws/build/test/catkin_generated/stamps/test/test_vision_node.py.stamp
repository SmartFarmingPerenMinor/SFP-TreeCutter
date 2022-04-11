#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import cv2
from cv_bridge import CvBridge

class Follower:
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        self.hz = rospy.Rate(1)
        self.image_sub = rospy.Subscriber('/ur10e/camera1/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('image_timer', Image, queue_size=10)
        rospy.on_shutdown(cv2.destroyAllWindows)

    def image_callback(self, msg):
        rospy.loginfo_once("Establishing img Callback")
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # new_image = self.image.copy()
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_OTSU)
        inverted_binary = ~binary
        contours, hierarchy = cv2.findContours(inverted_binary, 
        cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        with_contours = cv2.drawContours(self.image, contours, -1,(255,0,255),3)
        cv2.imshow('Detected contours', with_contours)
        cv2.waitKey(5)
        # Show the total number of contours that were detected
        # print('Total number of contours detected: ' + str(len(contours)))
    
    def start(self):
        rospy.loginfo("Starting image publishing")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            else:
                rospy.loginfo("Image not found!")
            self.hz.sleep()

def main():
    rospy.init_node('test_vision_node')
    follower = Follower()
    follower.start()
    rospy.spin()

if __name__ == "__main__":
    main()

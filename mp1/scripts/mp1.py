#!/usr/bin/env python3
import os, sys

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import imutils


class PedestrianDetector:
    def __init__(self):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def pedestrian_exists(self, image):
        image = imutils.resize(image, width=min(1050, image.shape[1]))
        # cv2.namedWindow("frame")
        # cv2.imshow("frame", image)
        # cv2.waitKey(0)
        (rects, weights) = self.hog.detectMultiScale(
            image, winStride=(4, 4), padding=(8, 8), scale=1.25
        )
        threshold = 1
        print(weights)
        for weight in weights:
            if weight > threshold:
                return True
        return False


class image_converter:
    def __init__(self):
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.bridge = CvBridge()
        self.detector = PedestrianDetector()
        self.image_sub = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.callback
        )
        self.flag = 0


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            person_exists = self.detector.pedestrian_exists(cv_image)
            if person_exists and self.flag < 3:
                print("Pedestrian in the frame, braking ...")
                self.enable_pub.publish(True)
                self.brake_pub.publish(f64_cmd=0.7, enable=True)
                print("Brake command published ...")
                time.sleep(2)
                self.brake_pub.publish(f64_cmd=1.0, enable=True)
                self.flag += 1
            elif person_exists:
                print("Pedestrian in the frame but already braked ...")
            else:
                self.flag = 0
                print("No pedestrian, safe to drive ...")
        except CvBridgeError as e:
            print(e)


rospy.init_node("image_converter", anonymous=True)

pub_enable = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)

ic = image_converter()
pub_enable.publish(True)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
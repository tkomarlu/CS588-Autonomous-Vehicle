#!/usr/bin/env python3
import os
import sys
import dlib
import roslib
import rospy
from std_msgs.msg import Bool, String, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import numpy as np
import imutils
from follow_pid_controller import follow_pid_controller

rospy.init_node("pedestrian_tracker", anonymous=True)
enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
accel_pub = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
steer_pub = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
gear_pub = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
brake_pub = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
enabled = False
enable_pub.publish(Bool(enabled))
accel_flag = False
gear_cmd = PacmodCmd()
accel_cmd = PacmodCmd()
brake_cmd = PacmodCmd()
steer_cmd = PositionWithSpeed()

class pedestrian_tracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.callback
        )

        self.tracker = dlib.correlation_tracker()
        self.initBB = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = imutils.resize(cv_image, width=600)
            (H, W) = frame.shape[:2]

            if self.initBB is not None:
                self.tracker.update(frame)
                bbox = self.tracker.get_position()
                x1, y1 = int(bbox.left()), int(bbox.top())
                x2, y2 = int(bbox.right()), int(bbox.bottom())
                cv2.rectangle(frame, (x1, y1), (x2, y2),
                              (0, 255, 0), 2)
                self.steer.speed_control(y2-y1)
                self.steer.steer_control((x1 + x2) / 2)
                info = [
                    ("Image Height", "{:.2f}".format((y2 - y1))),
                ]
                for (i, (k, v)) in enumerate(info):
                    text = "{}: {}".format(k, v)
                    cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Zed Camera", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                self.initBB = cv2.selectROI(
                    "Zed Camera", frame, fromCenter=False, showCrosshair=True)
                x, y, w, h = self.initBB
                points = [x, y, (x + w), (y + h)]
                self.tracker.start_track(frame, dlib.rectangle(*points))
                desired_x = x + (w / 2)
                desired_y = h
                self.steer = steer_pid_controller(desired_x, desired_y)

            elif key == ord("q"):
                sys.exit(0)

        except CvBridgeError as e:
            print(e)

ot = pedestrian_tracker()

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Exiting...")

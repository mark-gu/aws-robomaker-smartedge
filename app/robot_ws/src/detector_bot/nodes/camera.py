#!/usr/bin/python3

# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import logging
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen


class GetImage:
    def __init__(self):
        self.pub_img = rospy.Publisher('jetbot_camera/raw', Image, queue_size=1)

    def gstreamer_pipeline(
        self,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=60,
        flip_method=0
    ):
        return (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), '
            'width=(int)%d, height=(int)%d, '
            'format=(string)NV12, framerate=(fraction)%d/1 ! '
            'nvvidconv flip-method=%d ! '
            'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=(string)BGR ! appsink' %
            (capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )

    def show_camera(self):
        # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
        pipeline = self.gstreamer_pipeline(flip_method=0)
        print(pipeline)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            rate = rospy.rate(30)    # attempt to get fixed rate
            while not rospy.is_shutdown():
                ret_val, img = cap.read()
                resize_image = cv2.resize(img, (224, 224))
                msg_frame = CvBridge().cv2_to_imgmsg(resize_image, 'bgr8')
                self.pub_img.publish(msg_frame)
                rate.sleep()
            cap.release()
            cv2.destroyAllWindows()
        else:
            print('Unable to open camera')


if __name__ == '__main__':
    try:
        rospy.init_node("get_image")
        class_obj = GetImage()

        class_obj.show_camera()
        
    except Exception as e:
        logging.exception("An error occurred")
        raise e

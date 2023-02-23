#!/usr/bin/env python3
import os
import cv2                                          # to display the images
from cv_bridge import CvBridge, CvBridgeError       # to convert ros image messages to OpenCV images
import rospy                                        # ROS Python Interface
from sensor_msgs.msg import Image

class CamSubscriber(object):
    
    """
        The following code will provide the Camera data of the MiRO
        default: True for left, False for right
    """
    def __init__(self, default = True):
        rospy.init_node("cam_subscriber")
        # cv bridge is used for converting ros img data to data that can be used by open cv
        self.bridge = CvBridge()
        # part of the topic name that either changes the MiRo to use either the left or right camera
        if default == True:
            cam_topic = "/sensors/camr"
        else:
            cam_topic = "/sensors/caml"
        self.cam_data = None

        # subscriber
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + cam_topic, Image, self.callback)

    # callback function camr
    def callback(self, data):
        try:
            # convert ros img data to open cv images
            self.cam_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)


camera = CamSubscriber()
while not rospy.is_shutdown():
    if not camera.cam_data is None:
        cv2.imshow("Image window", camera.cam_data)
        cv2.waitKey(3)

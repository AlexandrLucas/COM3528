#!/usr/bin/env python3

import os
import rospy    # ROS Python interface
from std_msgs.msg import Float32MultiArray

class LightSubscriber(object):
    
    """
        The following code will provide information on lights
    """
    def __init__(self):
        rospy.init_node("light_subscriber")
        self.front = None
        self.right = None
        self.left = None
        self.back = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/light", Float32MultiArray, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.front = data.data[0]
        self.right = data.data[1]
        self.left = data.data[2]
        self.back = data.data[3]

light = LightSubscriber()
while not rospy.is_shutdown():
    # run the code to check the data from the subscriber
    toPrint = "Front: " + str(light.front) + "\nRight: " + str(light.right) + "\nLeft: " + str(light.left) + "\nBack: " + str(light.back)
    print(toPrint)
    rospy.sleep(0.5)    # to slow down the printing
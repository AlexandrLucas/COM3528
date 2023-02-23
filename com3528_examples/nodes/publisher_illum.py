#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
from std_msgs.msg import UInt32MultiArray

class IllumPublisher(object):
    
    """
        The following code will change color
    """
    def __init__(self):
        rospy.init_node("illumination_publisher")
        self.position = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

    # set color
    def set_illumination(self, red = 0, green = 0, blue = 0):
        # changing the rgb format into android format to be published in MiRo message
        color_change = UInt32MultiArray()
        color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        # six seperate leds in the miro
        color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
        self.illumination.publish(color_change)


illum = IllumPublisher()
while not rospy.is_shutdown():
    illum.set_illumination(red = 100, green = 100, blue = 100)

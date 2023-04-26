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
        self.front_left = None
        self.front_right = None
        self.rear_left = None
        self.rear_right = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/light", Float32MultiArray, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.front_left = data.data[0]
        self.front_right = data.data[1]
        self.rear_left = data.data[2]
        self.rear_right = data.data[3]

if __name__ == '__main__':
    light = LightSubscriber()
    rospy.sleep(0.5) # short wait to make sure everything is initialised
    while not rospy.is_shutdown():
        # Print the light sensors data
        # Data ranges from 0 (dark) to 1 (bright)
        toPrint = "Front Left: {}; Front Right: {}; Rear Left: {}; Rear Right: {}".format(
            light.front_left,
            light.front_right,
            light.rear_left,
            light.rear_right
        )
        print(toPrint)
        rospy.sleep(0.5) # to slow down the printing

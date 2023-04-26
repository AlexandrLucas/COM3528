#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from std_msgs.msg import UInt16

class TouchSubscriber(object):

    """
        The following code will provide information on touch
    """
    def __init__(self):
        rospy.init_node("touch_subscriber")

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_head,
        )
        rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_body,
        )
        self.touch_data = [None, None] # head and body

    def callback_head(self, touch_data):
        self.callback(touch_data, 0)

    def callback_body(self, touch_data):
        self.callback(touch_data, 1)

    # Unified callback for both touch sensor arrays
    def callback(self, touch_data, index):
        # the touch data is encoded as a bit array
        # formatted strings can handle this natively
        bit_str = "{0:014b}".format(touch_data.data)
        self.touch_data[index] = list(map(int, [*(bit_str)]))

        # Another way is to use bitwise shift
        #[touch_data.data & (1 << i) for i in range(14)]

if __name__ == '__main__':
    touch = TouchSubscriber()
    rospy.sleep(0.5) # short wait to make sure everything is initialised
    while not rospy.is_shutdown():
        # Print the touch sensors data
        ##TODO: make the output easier to map to actual sensor location
        print("Head sensor array: {}".format(touch.touch_data[0]))
        print("Body sensor array: {}".format(touch.touch_data[1]))
        print("-"*61)
        rospy.sleep(0.5) # to slow down the printing

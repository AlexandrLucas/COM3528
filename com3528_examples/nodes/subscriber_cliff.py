#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from std_msgs.msg import Float32MultiArray

class CliffSubscriber(object):

    """
        The following code will provide information on cliffs
    """
    def __init__(self):
        rospy.init_node("cliff_subscriber")
        self.lcliff = None
        self.rcliff = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/cliff", 
                                           Float32MultiArray, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.left = data.data[0]
        self.right = data.data[1]

if __name__ == '__main__':
    cliff = CliffSubscriber()
    rospy.sleep(0.5) # short wait to make sure everything is initialised
    while not rospy.is_shutdown():
        # Print the cliff sensors data
        # Data ranges from 0 (cliff) to 1 (surface)
        # Note that the sensor resolution is 1/15
        toPrint = 'Left Cliff: {}; Right Cliff: {}'.format(cliff.left, cliff.right)
        print(toPrint)
        rospy.sleep(0.5) # to slow down the printing
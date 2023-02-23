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
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/cliff", Float32MultiArray, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.lcliff = data.data[0]
        self.rcliff = data.data[1]

cliff = CliffSubscriber()
while not rospy.is_shutdown():
    # run the code to check the data from the subscriber
    toPrint = "Left Cliff: " + str(cliff.lcliff) + "\nRight Cliff: " + str(cliff.rcliff)
    print(toPrint)
    rospy.sleep(0.5)    # to slow down the printing
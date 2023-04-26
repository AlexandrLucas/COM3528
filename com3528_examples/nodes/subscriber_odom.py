#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from nav_msgs.msg import Odometry   # ROS odometry subsciriber
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomSubscriber(object):

    """
        The following code will provide information on the estimated odometry
        For best results, MiRo should be moving (e.g. 'publisher_movement.py')
    """
    def __init__(self):
        rospy.init_node("odom_subscriber")
        self.x = self.x0 = 0
        self.y = self.y0 = 0
        self.theta = self.theta0 = 0
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(self.topic_base_name + "/sensors/odom", Odometry, self.callback)

    # callback function is first used to translate the subscriber data
    # into euler coordinates which may then be used to update object instances
    def callback(self, data):
        orientation = data.pose.pose.orientation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (_, _, self.theta) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

    def reset(self):
        # Not a true reset, rather a change in frame of reference
        self.x0 = self.x
        self.y0 = self.y
        self.theta0 = self.theta

if __name__ == '__main__':
    odom = OdomSubscriber()
    rospy.sleep(0.5) # short wait to make sure everything is initialised
    # reset the odometry values at the start
    odom.reset()
    while not rospy.is_shutdown():
        # run the code to check the data from the subscriber
        toPrint = "Pose X: {}; Pose Y: {}; Pose yaw (theta): {}".format(
            odom.x - odom.x0,
            odom.y - odom.y0,
            odom.theta - odom.theta0
        )
        print(toPrint)
        rospy.sleep(0.5)

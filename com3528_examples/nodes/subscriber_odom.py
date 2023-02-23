#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from nav_msgs.msg import Odometry   # ROS odometry subsciriber
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomSubscriber(object):
    
    """
        The following code will provide information on the estimated odometry data of the MiRo
    """
    def __init__(self):
        rospy.init_node("odom_subscriber")
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(self.topic_base_name + "/sensors/odom", Odometry, self.callback)

    # callback function is first used to translate the subcriber data into euler coordinates which may then be used to update object instances 
    def callback(self, data):
        orientation = data.pose.pose.orientation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (_, _, self.yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

    def reset(self):
        self.x = self.y = pose_z = angular_x = angular_y = angular_z = 0
        publisher = rospy.Publisher(self.topic_base_name + "/sensors/odom", Odometry, queue_size=0)
        reset_values = Odometry()
        publisher.publish(reset_values)


odom = OdomSubscriber()
while not rospy.is_shutdown():
    # run the code to check the data from the subscriber
    toPrint = "position x: " + str(odom.x) + "\nposition y: " + str(odom.y) + "\ntheta: " + str(odom.yaw)
    print(toPrint)
    rospy.sleep(1)
    toPrint = "position x: " + str(odom.x) + "\nposition y: " + str(odom.y) + "\ntheta: " + str(odom.yaw)
    print(toPrint)
    odom.reset()

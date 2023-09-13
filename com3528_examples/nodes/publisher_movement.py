#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
from geometry_msgs.msg import TwistStamped

class MovementPublisher(object):

    """
        The following code will move the MiRo
    """
    def __init__(self):
        rospy.init_node("movement_publisher")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Clean-up
        rospy.on_shutdown(self.shutdown_hook)

    # linear is to move straight and angular is to turn
    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        vel_cmd = TwistStamped()
        # explanation of the messages in the document
        # message variable to move forward is done by linear.x
        vel_cmd.twist.linear.x = linear
        # message variable to turn is done by angular.z
        vel_cmd.twist.angular.z = angular
        self.vel_pub.publish(vel_cmd)

    def shutdown_hook(self):
        # Stop moving
        self.set_move_cmd()

if __name__ == '__main__':
    movement = MovementPublisher()

    while not rospy.is_shutdown():
        # Makes MiRo move in circles
        movement.set_move_cmd(linear=0.1, angular=0.6)
        rospy.sleep(0.02)

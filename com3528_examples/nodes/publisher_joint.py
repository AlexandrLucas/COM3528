#!/usr/bin/env python3
import os
import time
import rospy            # ROS Python interface
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import miro2 as miro  # Import MiRo Developer Kit library

class JointPublisher(object):

    """
        The following code will move the joints, cosmetic and kinematic
    """
    def __init__(self):
        rospy.init_node("joint_publisher")
        self.position = None
        self.start_time = time.time()    # time to be used for blink sample
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # kinematic joint publisher
        self.kinematic_pub = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        # cosmetic joint publisher
        self.cosmetic_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )
        # Clean-up
        rospy.on_shutdown(self.shutdown_hook)

    # movement for either tilt, lift, yaw or pitch
    # default values are calibration
    def set_move_kinematic(self,
                           tilt = miro.constants.TILT_RAD_CALIB, # not a real DOF
                           lift = miro.constants.LIFT_RAD_CALIB,
                           yaw = miro.constants.YAW_RAD_CALIB,
                           pitch = miro.constants.PITCH_RAD_CALIB
                           ):
        joint_cmd = JointState()
        joint_cmd.position = [tilt, lift, yaw, pitch]
        self.kinematic_pub.publish(joint_cmd)

    # movement for the tail, eye lid, ears
    # default values are calibration (except eyes)
    def set_move_cosmetic(self,
                          tail_pitch = 0,
                          tail_yaw = 0.5,
                          left_eye = 0, # No ptosis
                          right_eye = 0,
                          left_ear = 1.0/3.0,
                          right_ear = 1.0/3.0
                          ):
        joint_cmd = Float32MultiArray()
        joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(joint_cmd)

    def shutdown_hook(self):
        # Move joints to default positions
        self.set_move_kinematic()
        self.set_move_cosmetic()

if __name__ == '__main__':
    movement = JointPublisher()

    while not rospy.is_shutdown():
        current_time = time.time()
        # set angle will have oscillatory behaviors for blinking and head shake
        set_angle = np.sin(current_time - movement.start_time)
        movement.set_move_kinematic(yaw = set_angle)
        movement.set_move_cosmetic(left_ear=set_angle, right_ear=set_angle, left_eye = set_angle, right_eye = set_angle)
        rospy.sleep(0.02)


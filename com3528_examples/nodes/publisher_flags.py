#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
from std_msgs.msg import UInt32
import miro2 as miro

class FlagPublisher(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.flag_pub = rospy.Publisher(
            topic_base_name + "/control/flags", UInt32, queue_size=0
        )

    def set_flag(self,
                 persistent = True,
                 disable_speaker = False,
                 disable_wheel_control = False,
                 disable_opto = False,
                 disable_servo_power = False,
                 disable_i2c_fail_warn = False,
                 disable_cliff_reflex = False,
                 disable_kin_idle = False,
                 disable_translation = False
                 ):
        """
            The function takes a boolean value for each of the flag setting,
            as defined in MiRo constants.py, and publishes the result to 
            /miro/control/flags
        """
        self.flag_msg = UInt32()
        # default flag disable status leds
        self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS
        if persistent:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_PERSISTENT        
        if disable_speaker:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_SPEAKER
        if disable_wheel_control:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_WHEEL_CONTROL
        if disable_opto:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_OPTO
        if disable_servo_power:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_SERVO_POWER
        if disable_i2c_fail_warn:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_I2C_FAIL_WARN
        if disable_cliff_reflex:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
        if disable_kin_idle:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_KIN_IDLE
        if disable_translation:
            self.flag_msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_TRANSLATION
            
        self.flag_pub.publish(self.flag_msg)


if __name__ == '__main__':

    rospy.init_node("flag_publisher")

    flag = FlagPublisher()

    while not rospy.is_shutdown():
    # disable cliff reflex & speaker
        flag.set_flag(disable_cliff_reflex=True, disable_speaker=True)
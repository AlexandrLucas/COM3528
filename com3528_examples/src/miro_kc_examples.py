#!/usr/bin/env python3

"""
Simple script to test the 'push' method for MiRo's Kinematic Chain

IMPORTANT: Make sure to start node_body if running within the simulator 
(~/mdk/bin/deb64/node_body)

Further references:
flags -- miro2.constants
mdk/bin/shared/client_gui.py
mdk/bin/shared/example_kc.py
mdk/bin/shared/client_test.py push
"""

import os
import math
from random import choice

import rospy
import geometry_msgs
from std_msgs.msg import UInt32, Float32MultiArray

import miro2 as miro

# Create a new node
rospy.init_node("test_kc", anonymous=True)

# Individual robot name acts as ROS topic prefix
topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

# Prepare publishers for node_body
topic = topic_base_name + "/core/mpg/push"
pub_push = rospy.Publisher(topic, miro.msg.push, queue_size=0)

# Create the push message
msg_push = miro.msg.push()
msg_push.link = miro.constants.LINK_HEAD
msg_push.flags = miro.constants.PUSH_FLAG_VELOCITY

# There are 3 movement examples here to showcase
# All positions are relative, defined in the HEAD_FRAME
option = 3 # 1, 2, 3
print("Showing KC example {}".format(option))

t_now = 0.0
while not rospy.core.is_shutdown():

    # Prepare the push position
    # Movement looks most natural when started at the tip of the nose
    msg_push.pushpos = geometry_msgs.msg.Vector3(
        miro.constants.LOC_NOSE_TIP_X,
        miro.constants.LOC_NOSE_TIP_Y,
        miro.constants.LOC_NOSE_TIP_Z
        )
    
    if option == 1:
        # Case 1: Zero Vector (do nothing)
        tick = 0.02
        msg_push.pushvec = geometry_msgs.msg.Vector3(0, 0, 0)

    elif option == 2:
        # Case 2: (Don't) Look Up
        tick = 0.1
        msg_push.pushvec = geometry_msgs.msg.Vector3(
            miro.constants.LOC_NOSE_TIP_X, 
            miro.constants.LOC_NOSE_TIP_Y, 
            miro.constants.LOC_NOSE_TIP_Z + 1
            )
            
    elif option == 3:
        # Case 1: Turning the head left and right while moving forward
        tick = 0.02
        xk = math.sin(t_now * 0.25 * 2 * math.pi)
        # Prepare the push vector
        msg_push.pushvec = geometry_msgs.msg.Vector3(
            miro.constants.LOC_NOSE_TIP_X + 0.1, 
            miro.constants.LOC_NOSE_TIP_Y + 0.2 * xk, 
            miro.constants.LOC_NOSE_TIP_Z
            )

        
    # Publish the push message
    pub_push.publish(msg_push)

    # Update state
    rospy.sleep(tick)
    t_now += tick

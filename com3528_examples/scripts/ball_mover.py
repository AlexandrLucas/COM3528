#!/usr/bin/env python3
#

# Imports
##########################
import rospy  # ROS Python interface
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import miro2 as miro  # Import MiRo Developer Kit library
##########################

class Gazebo_Object():
        TICK = 0.02 # Main control loop period (secs)
        CUTOFF_TIME = 120 # Stop after this amount of seconds

        def __init__(self):
            self.gazebo_ns = "/gazebo_server" # Gazebo namespace;
            #self.gazebo_ns = "/gazebo" # Use this namespace if launched with ./launch_full.sh
            self.name = "miro_toy_ball" # Object name

            self.x0 = 0.6
            self.y0 = 0
            self.z0 = 0.1

            self.amp = 0.65
            self.speed_coeff = 0.5

if __name__ == "__main__":
    rospy.init_node('ball_mover')
    GO = Gazebo_Object()
    state_msg = ModelState()
    state_msg.model_name = GO.name
    state_msg.pose.position.x = GO.x0
    state_msg.pose.position.y = GO.y0
    state_msg.pose.position.z = GO.z0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service(GO.gazebo_ns + '/set_model_state')
    start_time = rospy.get_rostime()
    delta_t = 0
    while not rospy.core.is_shutdown():
        print(delta_t)
        rospy.sleep(GO.TICK)
        current_time = rospy.get_rostime()
        # Make the ball move in an oscillatory motion
        delta_t = (current_time - start_time).nsecs / (10**9) # secs
        phi = math.pi * delta_t / GO.speed_coeff
        state_msg.pose.position.y = GO.y0 + GO.amp*math.sin(phi)
        try:
            set_state = rospy.ServiceProxy(GO.gazebo_ns + '/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        if (current_time - start_time).secs > GO.CUTOFF_TIME:
            break

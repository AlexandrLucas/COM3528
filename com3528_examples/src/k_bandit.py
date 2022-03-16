#!/usr/bin/env python3
"""
Simple action selection mechanism inspired by the K-bandit problem

Initially, MiRo performs one of the following actions on random, namely: 
wiggle ears, wag tail, rotate, turn on LEDs and simulate a Braitenberg Vehicle.

While an action is being executed, stroking MiRo's head will reinforce it, while  
stroking MiRo's body will inhibit it, by increasing or reducing the probability 
of this action being picked in the future.

NOTE: The code was tested for Python 2 and 3
For Python 2 the shebang line is
#!/usr/bin/env python
"""

# Imports
##########################
import os
import numpy as np

import rospy  # ROS Python interface
from std_msgs.msg import (
    Float32MultiArray,
    UInt32MultiArray,
    UInt16,
)  # Used in callbacks
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control)

import miro2 as miro  # MiRo Developer Kit library
try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient:

    # Script settings below
    TICK = 0.02  # Main loop frequency (in secs, default is 50Hz)
    ACTION_DURATION = rospy.Duration(3.0)  # seconds
    VERBOSE = True  # Whether to print out values of Q and N after each iteration
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    def __init__(self):
        """
        Class initialisation
        """
        print("Initialising the controller...")

        # Get robot name
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("kbandit", anonymous=True)

        # Define ROS publishers
        self.pub_cmd_vel = rospy.Publisher(
            topic_root + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        self.pub_cos = rospy.Publisher(
            topic_root + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )
        self.pub_illum = rospy.Publisher(
            topic_root + "/control/illum", UInt32MultiArray, queue_size=0
        )

        # Define ROS subscribers
        rospy.Subscriber(
            topic_root + "/sensors/touch_head",
            UInt16,
            self.touchHeadListener,
        )
        rospy.Subscriber(
            topic_root + "/sensors/touch_body",
            UInt16,
            self.touchBodyListener,
        )
        rospy.Subscriber(
            topic_root + "/sensors/light",
            Float32MultiArray,
            self.lightCallback,
        )

        # List of action functions
        ##NOTE Try writing your own action functions and adding them here
        self.actions = [
            self.earWiggle,
            self.tailWag,
            self.rotate,
            self.shine,
            self.braitenberg2a,
        ]

        # Initialise objects for data storage and publishing
        self.light_array = None
        self.velocity = TwistStamped()
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.illum = UInt32MultiArray()
        self.illum.data = [
            0xFFFFFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
        ]

        # Utility enums
        self.tilt, self.lift, self.yaw, self.pitch = range(4)
        (
            self.droop,
            self.wag,
            self.left_eye,
            self.right_eye,
            self.left_ear,
            self.right_ear,
        ) = range(6)

        # Variables for Q-learning algorithm
        self.reward = 0
        self.punishment = 0
        self.Q = [0] * len(self.actions)  # Highest Q value gets to run
        self.N = [0] * len(self.actions)  # Number of times an action was done
        self.r = 0  # Current action index
        self.alpha = 0.7  # learning rate
        self.discount = 25  # discount factor (anti-damping)

        # Give it a sec to make sure everything is initialised
        rospy.sleep(1.0)

    def earWiggle(self, t0):
        print("MiRo wiggling ears")
        A = 1.0
        w = 2 * np.pi * 0.2
        f = lambda t: A * np.cos(w * t)
        i = 0
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.cos_joints.data[self.left_ear] = f(i)
            self.cos_joints.data[self.right_ear] = f(i)
            self.pub_cos.publish(self.cos_joints)
            i += self.TICK
            rospy.sleep(self.TICK)
        self.cos_joints.data[self.left_ear] = 0.0
        self.cos_joints.data[self.right_ear] = 0.0
        self.pub_cos.publish(self.cos_joints)

    def tailWag(self, t0):
        print("MiRo wagging tail")
        A = 1.0
        w = 2 * np.pi * 0.2
        f = lambda t: A * np.cos(w * t)
        i = 0
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.cos_joints.data[self.wag] = f(i)
            self.pub_cos.publish(self.cos_joints)
            i += self.TICK
            rospy.sleep(self.TICK)
        self.cos_joints.data[self.wag] = 0.0
        self.pub_cos.publish(self.cos_joints)

    def rotate(self, t0):
        print("MiRo rotating")
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 0.2
            self.pub_cmd_vel.publish(self.velocity)
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)

    def shine(self, t0):
        print("MiRo turning on LEDs")
        color = 0xFF00FF00
        i = 0
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            ic = int(np.mod(i, 6))
            ip = int(np.mod(i + 1, 6))
            self.illum.data[ic] = color
            self.illum.data[ip] = 0x00000000
            self.pub_illum.publish(self.illum)
            i += self.TICK
        self.illum.data[ic] = 0x00000000
        self.pub_illum.publish(self.illum)

    def braitenberg2a(self, t0):
        print("MiRo simulates a Braitenberg Vehicle")
        if self.light_array is None:
            wheel_speed = [0, 0]
        else:
            wheel_speed = [self.light_array[1], self.light_array[0]]
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = dr
            self.velocity.twist.angular.z = dtheta
            self.pub_cmd_vel.publish(self.velocity)
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)

    def touchHeadListener(self, data):
        """
        Positive reinforcement comes from stroking the head
        """
        if data.data > 0:
            self.reward += 1

    def touchBodyListener(self, data):
        """
        Negative reinforcement comes from stroking the body
        """
        if data.data > 0:
            self.punishment -= 1

    def lightCallback(self, data):
        """
        Get the frontal illumination
        """
        if data.data:
            self.light_array = data.data

    def loop(self):
        """
        Main loop
        """
        print("Starting the loop")
        while not rospy.core.is_shutdown():
            self.reward = 0
            self.punishment = 0
            # Select next action randomly or via Q score with equal probability
            if np.random.random() >= 0.5:
                print("Performing random action")
                self.r = np.random.randint(0, len(self.actions))
            else:
                print("Performing action with the highest Q score")
                self.r = np.argmax(self.Q)

            # Run the selected action and update the action counter N accordingly
            start_time = rospy.Time.now()
            self.N[self.r] += 1
            self.actions[self.r](start_time)
            if self.VERBOSE:
                print("Action finished, updating Q table")

            reward_strength = self.reward + self.punishment
            if reward_strength > 2.0:
                final_reward = 1.0
                print("This behaviour has been reinforced!")
            elif reward_strength < -2.0:
                final_reward = -1.0
                print("This behaviour has been inhibited!")
            else:
                final_reward = 0.0

            gamma = min(self.N[self.r], self.discount)
            self.Q[self.r] += self.alpha * (final_reward - self.Q[self.r]) / gamma
            if self.VERBOSE:
                print("Q scores are: {}".format(self.Q))
                print("N values are: {}".format(self.N))


# This is run when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop

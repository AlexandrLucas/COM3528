#!/usr/bin/env python3
import os
import rospy    # ROS Python interface
from sensor_msgs.msg import Range

class SonarSubscriber(object):

    """
        The following code will provide information on the sonar
    """
    def __init__(self):
        rospy.init_node("odom_subscriber")
        self.fov = None                 # the field of view that is used by the sensor
        self.min_range = None           # the minimum range set to be detected by the sensor
        self.max_range = None           # the maximum range set to be detected by the sensor
        self.range = None               # distance to the object within the set boundaries of the sensor
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/sonar", Range, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.fov = data.field_of_view
        self.min_range = data.min_range
        self.max_range = data.max_range
        self.range = data.range

if __name__ == '__main__':
    sonar = SonarSubscriber()
    rospy.sleep(0.5) # short wait to make sure everything is initialised
    while not rospy.is_shutdown():
        toPrint = "Field of view: {}; Minimum Range: {}; Maximum Range: {}; Range: {}".format(
            sonar.fov,
            sonar.min_range, # in reality min is 0.03m
            sonar.max_range, # inf means no echo received
            sonar.range
        )
        print(toPrint)
        rospy.sleep(0.5) # to slow down the printing
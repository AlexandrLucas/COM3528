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
        self.range = None               # the range of objects within the set boundaries of the sensor
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/sonar", Range, self.callback)

    # callback function used to update the object instances for this class
    def callback(self, data):
        self.fov = data.field_of_view
        self.min_range = data.min_range
        self.max_range = data.max_range
        self.range = data.range

sonar = SonarSubscriber()
while not rospy.is_shutdown():
    toPrint = "Field of view: " + str(sonar.fov) + "\nMinimum Range: " + str(sonar.min_range) + "\nMaximum Range: " + str(sonar.max_range) + "\nRange: " + str(sonar.range)
    print(toPrint)
    rospy.sleep(0.5)    # to slow down the printing
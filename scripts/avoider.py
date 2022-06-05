import rospy, os, math
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan

class avoider:
    def __init__(parent_name):
        self.p_name = parent_name
        pass
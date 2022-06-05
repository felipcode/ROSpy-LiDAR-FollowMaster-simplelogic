import rospy
import numpy as np
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as qt2e

class robot:
    def __init__(self, name):
        self.name = name
        self.max_lin = 0.22
        self.max_ang = 2.84
        self.pose = Pose()
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(1)
        self.pub = rospy.Publisher(self.name + "/cmd_vel", Twist, queue_size=1)
        self.sub_pose = rospy.Subscriber(self.name + "/odom", Odometry, self.track)   
        
    # tracks it's own odometry
    def track(self, msg):
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = qt2e([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw
    
    # se movimenta conforme o comando
    def move(self, msg):
        self.cmd_vel = msg
        self.pub.publish(self.cmd_vel)

    # fica em standby
    def run(self):
        rospy.spin()


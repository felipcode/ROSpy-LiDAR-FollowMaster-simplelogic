import rospy
import time
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, Pose2D
import tf
from tf.transformations import euler_from_quaternion as qt2e
from math import radians, copysign, sqrt, pow, pi, atan2,sin,cos
from tf.transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
# from robot import robot
# from lidar import lidar
import laser_geometry.laser_geometry as lg

size_angles = 360

class lidar:
    def __init__(self, parent_name):
        self.p_name = parent_name
        self.pub = rospy.Publisher(self.p_name + '/leader_pose', Pose, queue_size = 10)
        self.scann = LaserScan()
        self.track_frames = np.ones(size_angles)
        self.latest_ang = np.array([]) 
        self.latest_d = np.array([]) 
        self.m_pose = Pose(0,0,0, 0, 0)
        index = 0
        d=0
        ang = 0
        pass
    
    def listen(self, pose_flwr):
        # rospy.init_node("leaderPose", anonymous=True)
        self.sub = rospy.Subscriber( self.p_name + '/scan', LaserScan, self.callback, pose_flwr)
        # rospy.spin()

    def callback(self, msg, pose_flwr):
        self.current_time = rospy.Time.now()
        self.scann.header.stamp = self.current_time
        self.scann.header.frame_id = 'laser'
        self.scann.angle_min = -3.1415
        self.scann.angle_max = 3.1415
        self.scann.angle_increment = 0.00311202858575
        self.scann.time_increment = 4.99999987369e-05
        self.scann.range_min = 0.00999999977648
        self.scann.range_max = 32.0
        self.scann.ranges = msg.ranges[0:]
        self.scann.intensities = msg.intensities[0:]
        # self.size_angles = len(self.scann.ranges)
        # self.print(scann)
        out_pose = self.track_leader_pose(pose_flwr)
        # self.m_pose = self.track_leader_pose(pose_flwr)
        self.pub.publish(self.m_pose)
        pass
    
    def track_leader_pose(self, pose_flwr):             # subtração de fundo

        (x, y) = (0, 0)
        section_size = 360 -1
        buff_diff = []
        buff_lidar = []
        buff = []
        index_of_diff = []
        ranges_diff = []
        out_pose = Pose()
        index = []
        ang = 0
        idx = 0
        d=0.5

        #print(len(self.scann.ranges))
        if (len(self.scann.ranges) > 0):
            self.track_frames = np.row_stack(( self.track_frames, self.scann.ranges))
        

        # print(self.track_frames[0])
        # print(len(self.track_frames))
        # print(self.track_frames.size)
        ctt=0
        if (self.track_frames.size > 360 and self.track_frames.size <= 360*2 ):             # track the difference in the frames
            # print(self.track_frames)
            now = self.track_frames[1]
            before = self.track_frames[0]
            # print(now)
            for i in range(0, len(now)-2):
                diff = 0
                diff = pow((now[i] - before[i]), 2)
                # buff_diff.append(diff)
                if (diff >= 100):
                    buff_diff.append(diff)                                                  # holds the difference in the frames - no true logic - just for testing
                    index_of_diff.append(i)                                                 # holds indexes from the found significant distances
            
            self.track_frames = self.track_frames[1]


        # print(index_of_diff)
        # print(buff_diff)
        for i in index_of_diff:
            ranges_diff.append(self.scann.ranges[i])
            
            pass
        # [0:360] --> angles? -->  
        #
        #
        # ang = self.scann.angle_min * ( (index+1) - 180 )/180
        # d = 
        # pose_flwr.x ; pose_flwr.y ; pose_flwr.theta


        if(len(index_of_diff) > 0):                 # check the vector (index_of_diff) that stores data os the index of the background filter  --> frame difference
            for i in index_of_diff:
                index.append(i)

        
        if( len(index) > 0 ):                       # define latest angle and distance to master relative to the follower
            last = len(index)-1                                            # last read index
            # index = index[x:]
            e = int(last - last/4)
            ax = index[e]                                               # holds some index in the indexes vector //holds first pos
            # ang = (self.scann.angle_min * ( (ax+1) - 180 )/180)   /2      # angle convertion
            ang = (ax+1)/180
            self.latest_ang  = np.append(self.latest_ang, ang)                                     # angle buffer
            print("angle: ", ang)
        
            if (len(self.scann.ranges) > 0 ):
                # d = self.scann.ranges[ax]                               # distance  -- ranges vector of chosen index (ax)
                d = 0.6
                self.latest_d  = np.append(self.latest_d, d)
                # self.latest_d.append(d)                                 # distance buffer
            

            if (len(self.latest_d)> 1 and len(self.latest_ang) > 1):     # updates latests reads
                self.latest_d = self.latest_d[1:]
                self.latest_ang = self.latest_ang[1:]
                ang, d = self.latest_ang[0], self.latest_d[0]
                print("angle and dist: ",ang, d)
                y, x = d*cos(ang) , d*sin(ang)
                x, y = pose_flwr.x + x , pose_flwr.y + y


                # print(pose_flwr.x , pose_flwr.y)
                # print(x, y)
                out_pose.x = x
                out_pose.y = y
                out_pose.theta = 0
                
                self.m_pose = out_pose
                # print(self.m_pose)
                return self.m_pose
                
            index = index[last:]       

        # print(self.m_pose)
        return self.m_pose
        # rospy.spin()
    def update_tracking(self):    
        pass





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




class follower:
    def __init__(self, leader, follower):
        rospy.init_node("ldr_flwr", anonymous=False)
        self.folwr = robot(follower)
        self.leader = robot(leader)
        self.machinist = rospy.Rate(10)
        self.min_radius = 0.5
        # self.sub1 = rospy.Subscriber("/cmd_vel", Twist, self.teleop)
        self.sensor = lidar(follower)
        self.sensor.listen(self.folwr.pose)
        # self.aux_master_pose = self.sensor.track_leader_pose()

        #self.sub_master_pose = self.sensor.track_leader_pose(self.folwr.pose)
        

    # intercepta o comando de teleop e move o líder

    def master_get_pose(self, msg):

        pass
    def teleop(self, msg):
        self.leader.move(msg)
    # obtém o vetor de posição do vagão em relação ao líder
    def relPos(self, lidar, follower, leader):
        # rospy.spin()
        # ref = leader.pose
        ref = lidar.m_pose
        print("master pose: \n",ref)
        
        flw = follower.pose
        module = np.sqrt((ref.x - flw.x)**2 + (ref.y - flw.y)**2)
        angle = np.arctan2(ref.y - flw.y,  ref.x - flw.x) - flw.theta
        return (module, angle)
    
    # ação de controle baseada no vetor de posição relativa
    def control(self, rel_pos, kp=(1.5, 6)):
        ctrl_vel = Twist()
        stop = (rel_pos[0] > self.min_radius)
        # ação de controle linear
        ctrl_vel.linear.x  = kp[0] * rel_pos[0] * stop
        ctrl_vel.linear.y = 0
        ctrl_vel.linear.z = 0
        # ação de controle angular
        ctrl_vel.angular.z = kp[1] * rel_pos[1] * stop
        ctrl_vel.angular.x = 0
        ctrl_vel.angular.y = 0

        #SATURACAO
        # velocidade linear máxima
        if abs(ctrl_vel.linear.x) > self.folwr.max_lin:
            ctrl_vel.linear.x = self.folwr.max_lin * np.sign(ctrl_vel.linear.x)
        # velocidade angular máxima
        if abs(ctrl_vel.angular.z) > self.folwr.max_ang:
            ctrl_vel.angular.z = self.folwr.max_ang * np.sign(ctrl_vel.angular.z)
        return ctrl_vel

    def run(self):
        while not rospy.is_shutdown():
            (d, ang) = self.relPos(self.sensor, self.folwr, self.leader)

            cmd = self.control((d, ang))
            self.folwr.move(cmd)
            
        
        self.machinist.sleep()


if __name__ == "__main__":
    try:
        maluk = follower('tb3_0', 'tb3_1')
        maluk.run()

    except rospy.ROSInterruptException:
        pass
        pass









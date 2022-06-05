#! /usr/bin/env python

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import numpy as np

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




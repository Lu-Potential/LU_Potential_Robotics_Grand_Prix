#!/usr/bin/env python
import rospy
import math
import csv
import tf
import numpy as np
from os.path import expanduser
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

obj_dect=1
flag=0
circle_r=4

home = expanduser('~')
filepath=home+'/rcws/logs/wp-berlin-max-2.csv' 
with open(filepath,'r') as csvfile:
    data = list(csv.reader(csvfile))
    waypoints_tmp = np.asarray(data)  
    waypoints = waypoints_tmp.astype(np.float32)

filepath=home+'/rcws/logs/wp-smooth-skeleton.csv'
with open(filepath,'r') as csvfile_skeleton:
    data_skeleton = list(csv.reader(csvfile_skeleton))
    waypoints_tmp_skeleton = np.asarray(data_skeleton)  
    waypoints_skeleton = waypoints_tmp_skeleton.astype(np.float32)


wp_xs_s = waypoints_skeleton[:,0]
wp_ys_s = waypoints_skeleton[:,1]
wp_len_s=len(wp_xs_s)
current_wp_s_ego=0
current_wp_s_opp=0
slow_down=1

start=0
wp_xs = waypoints[:,0]
wp_ys = waypoints[:,1]
wp_dir = waypoints[:,2]
wp_vel = waypoints[:,3]
wp_loka = waypoints[:,4]
wp_str = waypoints[:,5]
wp_len = len(wp_xs)
current_wp=0
Slow_down_param=0.12
VELOCITY=6
STR=1.2
#LOOKAHEAD_DISTANCE = 0.8**2
LOOKAHEAD_DISTANCE = 2**2
dis_threshold=0.5
dis_threshold_2=0.5

Car_x=0
Car_y=0
Car_dir_euler=[0,0,0]
bubble_size=0.5
protection_dis=0.3

dis2wp=10

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        odom_topic = '/lu/odom'
        opp_odom_topic = '/lu/opp_odom'
        drive_topic = '/lu/drive'
        scan_topic='/lu/scan'
        map_topic='/map'
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.opp_odom_sub = rospy.Subscriber(opp_odom_topic, Odometry, self.opp_odom_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=10)
        self.drive_pub =  rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def map_callback(self, map_msg):
        global flag
        if flag==0:
            wall_num=0
            map_data=[x for x in map_msg.data]
            for ii in range(len(map_data)):
                if map_data[ii]>0:
                    wall_num=wall_num+1
            if wall_num<252052 and wall_num>252040:
                flag=2
            else:
                flag=1
    def odom_callback(self, odom_msg):
        global flag
        global VELOCITY
        if flag==0 or flag==2:
	    global wp_xs
	    global wp_ys
	    global wp_dir
	    global wp_vel
	    global wp_loka
	    global wp_str
	    global wp_len
	    global current_wp
            global Car_x
            global Car_y
            global Car_dir_euler
            global LOOKAHEAD_DISTANCE
            global dis2wp
            global STR
	    Car_x = float(odom_msg.pose.pose.position.x)
	    Car_y = float(odom_msg.pose.pose.position.y)
            global start
            if start == 0 and Car_y > 0:
                current_wp=0
                start =2
                filepath=home+'/rcws/logs/wp-berlin-max-3.csv'
                with open(filepath,'r') as csvfile:
                    data = list(csv.reader(csvfile))
                    waypoints_tmp = np.asarray(data)  
                    waypoints = waypoints_tmp.astype(np.float32)
                    wp_xs = waypoints[:,0]
                    wp_ys = waypoints[:,1]

            if start == 0 and Car_y < 0:
                current_wp=42
                start =1
            dis2wp=(Car_x-wp_xs[current_wp])**2+(Car_y-wp_ys[current_wp])**2
            while dis2wp<LOOKAHEAD_DISTANCE:
                current_wp=current_wp+1
                if current_wp>=wp_len:
                    current_wp=7
                dis2wp=(Car_x-wp_xs[current_wp])**2+(Car_y-wp_ys[current_wp])**2
            VELOCITY=wp_vel[current_wp]
            STR=wp_str[current_wp]
        #LOOKAHEAD_DISTANCE=wp_loka[current_wp]**2

            Car_dir_x=odom_msg.pose.pose.orientation.x
            Car_dir_y=odom_msg.pose.pose.orientation.y
            Car_dir_z=odom_msg.pose.pose.orientation.z
            Car_dir_w=odom_msg.pose.pose.orientation.w

            Car_dir_q = (Car_dir_x,Car_dir_y,Car_dir_z,Car_dir_w)
            Car_dir_euler = tf.transformations.euler_from_quaternion(Car_dir_q)
        else:
            return 0

    def opp_odom_callback(self, opp_odom_msg):
        global wp_xs_s
        global wp_ys_s
        global wp_len_s
        global current_wp_s_ego
        global current_wp_s_opp
        global slow_down
        global current_wp
        global Car_x
        global Car_y
        global start
        
        dis2wp_s_c=(Car_x-wp_xs_s[current_wp_s_ego])**2+(Car_y-wp_ys_s[current_wp_s_ego])**2
        next_wp_s_ego=current_wp_s_ego+1
        if next_wp_s_ego>=wp_len_s:
            next_wp_s_ego=0
        dis2wp_s_n=(Car_x-wp_xs_s[next_wp_s_ego])**2+(Car_y-wp_ys_s[next_wp_s_ego])**2
        while dis2wp_s_c>dis2wp_s_n:
            current_wp_s_ego=next_wp_s_ego
            dis2wp_s_c=(Car_x-wp_xs_s[current_wp_s_ego])**2+(Car_y-wp_ys_s[current_wp_s_ego])**2
            next_wp_s_ego=current_wp_s_ego+1
            if next_wp_s_ego>=wp_len_s:
                next_wp_s_ego=0
            dis2wp_s_n=(Car_x-wp_xs_s[next_wp_s_ego])**2+(Car_y-wp_ys_s[next_wp_s_ego])**2

        Car_x_opp = float(opp_odom_msg.pose.pose.position.x)
        Car_y_opp = float(opp_odom_msg.pose.pose.position.y)
        
        dis2wp_s_c=(Car_x_opp-wp_xs_s[current_wp_s_opp])**2+(Car_y_opp-wp_ys_s[current_wp_s_opp])**2
        next_wp_s_opp=current_wp_s_opp+1
        if next_wp_s_opp>=wp_len_s:
            next_wp_s_opp=0
        dis2wp_s_n=(Car_x_opp-wp_xs_s[next_wp_s_opp])**2+(Car_y_opp-wp_ys_s[next_wp_s_opp])**2
        while dis2wp_s_c>dis2wp_s_n:
            current_wp_s_opp=next_wp_s_opp
            dis2wp_s_c=(Car_x_opp-wp_xs_s[current_wp_s_opp])**2+(Car_y_opp-wp_ys_s[current_wp_s_opp])**2
            next_wp_s_opp=current_wp_s_opp+1
            if next_wp_s_opp>=wp_len_s:
                next_wp_s_opp=0
            dis2wp_s_n=(Car_x_opp-wp_xs_s[next_wp_s_opp])**2+(Car_y_opp-wp_ys_s[next_wp_s_opp])**2
        behind=current_wp_s_opp-current_wp_s_ego
        if behind<0:
            behind = behind+wp_len_s
        if behind>0 and behind<50:
            slow_down=behind*0.008+0.6
        else:
            slow_down=1
        if start == 1 or current_wp < 7:
            slow_down=1
        #print(current_wp_s_ego,current_wp_s_opp,slow_down)


    def scan_callback(self, scan_msg):
        global flag
        global dis_threshold
        global VELOCITY
        if flag==1:
            global circle_r
            global bubble_size
            proc_ranges=[x for x in scan_msg.ranges]
            scan_min_idx=int((-1.57-scan_msg.angle_min)/scan_msg.angle_increment)
            scan_max_idx=int((1.57-scan_msg.angle_min)/scan_msg.angle_increment)
            idx=scan_min_idx
            while idx < scan_max_idx:
                if proc_ranges[idx+1]-proc_ranges[idx] > dis_threshold:
                    idx_size=int(bubble_size/(proc_ranges[idx]*scan_msg.angle_increment))
                    for j in range(idx+1,idx+idx_size):
                        proc_ranges[j]=proc_ranges[idx]
                    idx=idx+idx_size
                elif proc_ranges[idx]-proc_ranges[idx+1]>dis_threshold:
                    idx_size=int(bubble_size/(proc_ranges[idx+1]*scan_msg.angle_increment))
                    for j in range(idx-idx_size,idx+1):
                        proc_ranges[j]=proc_ranges[idx+1]
                idx=idx+1
            depth=0
            idx=scan_min_idx
            best_idx=scan_min_idx
            while idx < scan_max_idx:
                if proc_ranges[idx] > depth:
                    best_idx=idx
                    depth=proc_ranges[idx]
                idx=idx+1
            for idx in range(len(proc_ranges)):
                if proc_ranges[idx]<0.3:
                    best_idx=int(len(proc_ranges)/2)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.drive.steering_angle = (best_idx*scan_msg.angle_increment+scan_msg.angle_min)*0.6
            drive_msg.drive.speed = 3
            self.drive_pub.publish(drive_msg)
        elif flag==2:
                
	    global wp_xs
	    global wp_ys
            global wp_str
            global wp_len
	    global current_wp

            global Car_x
            global Car_y
            global Car_dir_euler

            global STR
            global dis2wp
            global dis_threshold_2
            global protection_dis
            global slow_down


            Car_dir_v_x=math.cos(Car_dir_euler[2])
            Car_dir_v_y=math.sin(Car_dir_euler[2])

            v1 = np.array([Car_dir_v_x,Car_dir_v_y])
            v2 = np.array([wp_xs[current_wp]-Car_x,wp_ys[current_wp]-Car_y])
            v1 = v1/np.linalg.norm(v1)
            v2 = v2/np.linalg.norm(v2)
            v = np.cross(v1,v2)

            angle = math.asin(v)*STR

        
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = VELOCITY*slow_down
            self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node('Lehigh_race_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()

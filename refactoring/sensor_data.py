#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from autocarz.msg import Erp42State
from math import sqrt, atan2, sin, tan, cos, log, floor, pi

class LidarSensor:
    """
    Handles Lidar sensor data.
    """
    def __init__(self):
        # Subscribers for Lidar data
        rospy.Subscriber("vlp/distance_front", Float32, self.get_vlp_front)
        rospy.Subscriber("vlp/distance_side_front", Float32, self.get_vlp_side_front)
        rospy.Subscriber("vlp/distance_side_back", Float32, self.get_vlp_side_back)
        rospy.Subscriber("livox/distance_front", Float32, self.get_livox_front) 
        rospy.Subscriber("livox/distance_side_front", Float32, self.get_livox_side_front) 
        rospy.Subscriber("livox/distance_side_back", Float32, self.get_livox_side_back)
        rospy.Subscriber("livox/max_roi_dis_in", Float32, self.get_livox_roi_in) 
        rospy.Subscriber("livox/max_roi_dis_out", Float32, self.get_livox_roi_out)        

        # Lidar data variables
        self.vlp_front = 0.0
        self.vlp_side_front = 0.0
        self.vlp_side_back = 0.0
        self.livox_front = 0.0
        self.livox_side_front = 0.0
        self.livox_side_back = 0.0
        self.livox_roi_in = 0.0
        self.livox_roi_out = 0.0
        self.VLP_DISTANCE = 15.0

    def get_vlp_front(self, data):
        self.vlp_front = data.data

    def get_vlp_side_front(self, data):
        self.vlp_side_front = data.data

    def get_vlp_side_back(self, data):
        self.vlp_side_back = data.data
        
    def get_livox_front(self,data):
        self.livox_front = data.data + 0.6

    def get_livox_side_front(self,data):
        self.livox_side_front = data.data + 0.6

    def get_livox_side_back(self,data):     # 만들어두긴했는데 안씀
        self.livox_side_back = data.data  

    def get_livox_roi_in(self,data):        # waypoint x기준계산으로 바꿈 +0.6함
        self.livox_roi_in = data.data  + 0.6
    
    def get_livox_roi_out(self,data):
        self.livox_roi_out = data.data + 0.6
        
    def lidar_data(self, lane):
        ''' 
        livox, vlp 의 모든 토픽을 종합하여 아래 3가지 변수를 채움
        lidar_d_front
        lidar_d_side_front
        lidar_d_side_back
        '''
        if lane==1:
            max_roi_dis_front = self.livox_roi_in
            max_roi_dis_side = self.livox_roi_out
        else:
            max_roi_dis_front = self.livox_roi_out
            max_roi_dis_side = self.livox_roi_in

        # distance front
        if max_roi_dis_front < self.livox_front : 
            if self.vlp_front <= self.VLP_DISTANCE:
                lidar_d_front = self.vlp_front
            else:
                if max_roi_dis_front < self.VLP_DISTANCE:
                    lidar_d_front = self.VLP_DISTANCE
                else:
                    lidar_d_front =max_roi_dis_front
        else : 
            lidar_d_front = min( self.vlp_front, self.livox_front)

        # distance side front
        if max_roi_dis_side < self.livox_side_front : 
            if self.vlp_side_front <= self.VLP_DISTANCE:
                lidar_d_side_front = self.vlp_side_front
            else:
                if max_roi_dis_side < self.VLP_DISTANCE:
                    lidar_d_side_front = self.VLP_DISTANCE
                else:
                    lidar_d_side_front =max_roi_dis_side
        else : 
            lidar_d_side_front = min(self.vlp_side_front, self.livox_side_front)

        # distance side back
        lidar_d_side_back = self.vlp_side_back
        
        return lidar_d_front, lidar_d_side_front, lidar_d_side_back

class GPS:
    """
    Handles GPS data.
    """
    def __init__(self):
        # Subscribers for GPS data
        rospy.Subscriber("/ublox/fix", NavSatFix, self.getGPS)

        # GPS data variables
        self.odom = Odometry()
        self.gps_status = 0
        self.gps_covariance = 0.0
        self.pastPos = [0.0, 0.0]
        self.checkGPS = False
        self.checkHeading = False
        self.last_gps_in = rospy.Time.now()

    def getGPS(self, data):
        gpsX, gpsY = self.latlong2xy(data.latitude, data.longitude)
        self.odom.header.frame_id = "map"
        self.odom.pose.pose.position.x = gpsX
        self.odom.pose.pose.position.y = gpsY
        self.odom.pose.pose.position.z = 0.0 
        self.last_gps_in = rospy.Time.now()
        self.gps_status = data.status.status
        self.gps_covariance = data.position_covariance[0]

        if self.checkGPS and 1.0 < self.velocity and self.velocity < 50.0:
            dx = gpsX - self.pastPos[0]
            dy = gpsY - self.pastPos[1]
            dist = sqrt(dx**2+dy**2)
            
            if dist > 0.1:                      
                self.heading=atan2(dy, dx)
                self.odom.pose.pose.orientation.x = 0.0
                self.odom.pose.pose.orientation.y = 0.0
                self.odom.pose.pose.orientation.z = sin(self.heading*0.5)
                self.odom.pose.pose.orientation.w = cos(self.heading*0.5)
                self.checkGPS = False
                self.checkHeading = True

        if not self.checkGPS:
            self.pastPos = [gpsX, gpsY]
            self.checkGPS = True
            
        if not self.checkHeading:
            print("No heading")            

    def latlong2xy(self, lat, long):
        RE = 6371.00877     # 지구 반경(km)
        GRID = 0.0000005    # 격자 간격(km), 0.5mm단위 
        SLAT1 = 30.0
        SLAT2 = 60.0
        OLON = 126.0
        OLAT = 38.0
        XO = 43
        YO = 136

        DEGRAD = pi / 180.0
        re = RE / GRID
        slat1 = SLAT1 * DEGRAD
        slat2 = SLAT2 * DEGRAD
        olon = OLON * DEGRAD
        olat = OLAT * DEGRAD

        sn = tan(pi * 0.25 + slat2 * 0.5) / tan(pi * 0.25 + slat1 * 0.5)
        sn = log(cos(slat1) / cos(slat2)) / log(sn)
        sf = tan(pi * 0.25 + slat1 * 0.5)
        sf = pow(sf, sn) * cos(slat1) / sn
        ro = tan(pi * 0.25 + olat * 0.5)
        ro = re * sf / pow(ro, sn)

        ra = tan(pi * 0.25 + (lat) * DEGRAD * 0.5)
        ra = re * sf / pow(ra, sn)

        theta = long * DEGRAD - olon
        if theta > pi:
            theta -= 2.0 * pi
        if theta < -pi:
            theta += 2.0 * pi
        theta *= sn
        rs_x = floor(ra * sin(theta) + XO + 0.5)
        rs_y = floor(ro - ra * cos(theta) + YO + 0.5)
        first_xx = -93464 
        first_yy = 130046 
        rs_x = rs_x/2000 + first_xx
        rs_y = rs_y/2000 + first_yy

        return rs_x, rs_y

    def gps_data(self):
        return self.odom, self.heading, self.gps_status, self.gps_covariance, self.checkGPS, self.checkHeading, self.last_gps_in
    
class VehicleState:
    """
    Handles vehicle state data like velocity and steering.
    """
    def __init__(self):
        # Subscribers for vehicle state data
        rospy.Subscriber("/erp42/state", Erp42State, self.get_vehicle_state)

        # Vehicle state variables
        self.velocity = 0.0
        self.steering = 0.0

    def get_vehicle_state(self, data):
        velocity = data.velocity    # data.velocity 0~20/3.6 m/s  --> self.velocity 0~20 km/h, self.target_velocity 0~20 km/h

        if velocity > 100 or velocity < 0:
            self.velocity = 0.0
        else:
            self.velocity = velocity
        
        self.steering = data.steering
        
        self.checkState = True
        
    def vehicle_data(self):
        return self.velocity, self.steering, self.checkState

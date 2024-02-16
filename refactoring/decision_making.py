#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import sqrt, sin, cos, atan2
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from autocarz.msg import CtrlCmd, WaypointInfo
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    """
    Handles path planning for the autonomous vehicle.
    """
    def __init__(self, globalPathIn, globalPathOut):
        self.curWaypointIndxIn = 0
        self.curWaypointIndxOut = 0
        self.globalPathIn = globalPathIn
        self.globalPathOut = globalPathOut
        self.waypointInfo = WaypointInfo()
        self.waypointInfoPub = rospy.Publisher('/waypointInfo', WaypointInfo, queue_size=1)
        self.distance_log_in = rospy.Publisher('distance_error_in', Float32, queue_size=1)
        self.distance_log_out = rospy.Publisher('distance_error_out', Float32, queue_size=1)
        
    def localization(self, odom, lane):
        """
        Publish distance errors for debugging and monitoring.
        """
        self.waypointInfo.lane = lane
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        
        temp = Float32()
        minDist = float("inf")
        for i in range(len(self.globalPathIn.poses)):
            dx = curX-self.globalPathIn.poses[i].pose.position.x
            dy = curY-self.globalPathIn.poses[i].pose.position.y
            dist = sqrt(dx*dx+dy*dy)
            if dist < minDist:
                curWaypointIndxIn = i
                minDist = dist
        temp.data = minDist
        self.distance_log_in.publish(temp)
        self.waypointInfo.indxIn = curWaypointIndxIn
        
        temp = Float32()
        minDist = float("inf")
        for i in range(len(self.globalPathOut.poses)):
            dx = curX-self.globalPathOut.poses[i].pose.position.x
            dy = curY-self.globalPathOut.poses[i].pose.position.y
            dist = sqrt(dx*dx+dy*dy)
            if dist < minDist:
                curWaypointIndxOut = i
                minDist = dist
        temp.data = minDist
        self.distance_log_out.publish(temp)
        self.waypointInfo.indxOut = curWaypointIndxOut

        self.waypointInfoPub.publish(self.waypointInfo)
        
        return curWaypointIndxIn, curWaypointIndxOut

class SpeedDecision:
    """
    Manages the speed of the vehicle based on various factors.
    """
    def __init__(self):
        self.ctrlPub = rospy.Publisher('/ctrlCmd', CtrlCmd, queue_size=1)
        
    def control_speed(self, lidar_d_front, path_idx, velocity_scale_side):

        add_speed = 0.0
        ref_velocity = 18.0 + add_speed

        if path_idx == 0:
            velocity_scale_path = (18.0+add_speed)/ref_velocity
        elif path_idx == 1:
            velocity_scale_path = (16.0+add_speed)/ref_velocity    
        elif path_idx == 2:
            velocity_scale_path = 14.0/ref_velocity
        elif path_idx == 3:
            velocity_scale_path = (18.0+add_speed)/ref_velocity
        elif path_idx == 4:
            velocity_scale_path = 14.0/ref_velocity
        elif path_idx == 5:
            velocity_scale_path = (18.0+add_speed)/ref_velocity
        elif path_idx == 6:
            velocity_scale_path = 14.0/ref_velocity
        elif path_idx == 7:
            velocity_scale_path = 14.0/ref_velocity
        elif path_idx == 8:
            velocity_scale_path = 8.0/ref_velocity
        else:
            print("path_path_index_error")
        
        min_v = 6.0 * velocity_scale_side
        max_v = 18.0 * velocity_scale_side * velocity_scale_path

        min_d = 8.0
        max_d = 25.0

        criteria_d = (min_d + max_d) / 2.0
        criteria_v = (min_v + (max_v - min_v)*(criteria_d-min_d)/(max_d-min_d)) * velocity_scale_side * velocity_scale_path

        p_gain = (max_v-min_v) / (max_d-min_d)
        i_gain = 0.1
        d_gain = 0.0
        controlTime = 0.0333 #30Hz
        prev_error = 0.0
        i_control = 0.0
        
        brake = 0

        # Velocity(pid control)
        if lidar_d_front < min_d:
            target_velocity = 0
            brake = 100

        elif min_d <= lidar_d_front < max_d:
            error = lidar_d_front - criteria_d

            p_control = p_gain*error
            i_control += i_gain*error*controlTime
            d_control = d_gain*(error-prev_error)/controlTime

            output = p_control+i_control+d_control
            prev_error = error

            result_v = criteria_v + output

            if (0 < result_v <= max_v):
                target_velocity = result_v
            elif result_v <= 0:
                target_velocity = 0  
            else:
                target_velocity = max_v

            if target_velocity < 5.0:
                target_velocity = 5.0
            
        else:
            target_velocity = max_v
        
        return target_velocity, brake
    
    def gps_status_checker(self, gps_status, gps_covariance, last_gps_in, path_path_index):
        if rospy.Time.now() - last_gps_in > rospy.Duration(0.5):
            rospy.loginfo("GPS TOPIC TIME ERROR")
            return False
        
        if gps_status != 2:         
            for i in [0,1,3,5,8]:
                if i == path_path_index:
                    if gps_covariance < 0.01:
                        rospy.loginfo("GPS STATUS 0 GO " + str(gps_covariance))
                        return True
                else:
                    if gps_covariance < 0.005:
                        rospy.loginfo("GPS STATUS 0 GO " + str(gps_covariance))
                        return True
            rospy.loginfo("GPS STATUS 0 STOP " + str(gps_covariance))
            
            return False
        
        #rospy.loginfo("GPS OKAY")
        return True
        
    def isGPSerror(self, gps_status, gps_covariance, last_gps_in, path_path_index):
        if(not self.gps_status_checker(gps_status, gps_covariance, last_gps_in, path_path_index)):
            self.ctrl_msg.longlCmdType = 2
            self.ctrl_msg.velocity = 0.0
            self.ctrl_msg.brake = 100
            self.ctrlPub.publish(self.ctrl_msg)      
            return True
        
        else:
            return False

class SteeringDecision:
    """
    Decision the steering angle of the vehicle.
    """
    def __init__(self, numGlobalPathIn, numGloablPathOut, globalPathIn, globalPathOut):
        self.carLength = 1.3
        self.lfdPub = rospy.Publisher('/lfd', Odometry, queue_size=1)
        self.pathInfo = {1 : [numGlobalPathIn, globalPathIn], 2 : [numGloablPathOut, globalPathOut]}

    def purePursuit(self, odom, lane, velocity, heading, path_idx, curWaypointIndxIn, curWaypointIndxOut):
        velocity /= 3.6   # m/s
        curX = odom.pose.pose.position.x
        curY = odom.pose.pose.position.y
        bound = 150

        lfdPointidx = 1
        if(path_idx == 0 or path_idx == 1):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.3841+5.85
        elif(path_idx == 2):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.675+2.875
        elif(path_idx == 3):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.3841+5.85
        elif(path_idx == 4):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.675+2.875
        elif(path_idx == 5):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.3841+5.85
        elif(path_idx == 6):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.675+2.875
        elif(path_idx == 7):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*1.035+0.275
        elif(path_idx == 8):
            if(velocity<6/3.6):
                velocity=6/3.6
            lfd=(velocity)*0.3841+5.85
        else:
            print("path path_index error")

        curWaypoint = curWaypointIndxIn if lane == 1 else curWaypointIndxOut
        if curWaypoint + bound >= self.pathInfo[lane][0]:
            localWaypoints = range(curWaypoint, self.pathInfo[lane][0]) + range(0, bound-(self.pathInfo[lane][0]-curWaypoint))
        else:
            localWaypoints = range(curWaypoint, curWaypoint + bound)

        for i in localWaypoints:
            dx = self.pathInfo[lane][1].poses[i].pose.position.x - curX
            dy = self.pathInfo[lane][1].poses[i].pose.position.y - curY
            rx = cos(heading) * dx + sin(heading) * dy
            ry = sin(heading) * dx - cos(heading) * dy

            if rx > 0.0:
                dist = sqrt(rx*rx+ry*ry)
                if dist >= lfd:
                    theta = atan2(ry, rx)
                    steering = -atan2(2 * self.carLength * sin(theta), lfd)
                    lfdPointidx = i
                    break

        # Look Forawrd Distance Publish
        lfdPoint = Odometry()
        lfdPoint.header.frame_id = "map"
        lfdPoint.pose.pose.position.z = 0.0
        lfdPoint.pose.pose.orientation.x = 0.0
        lfdPoint.pose.pose.orientation.y = 0.0
        lfdPoint.pose.pose.orientation.z = 0.0
        lfdPoint.pose.pose.orientation.w = 1.0
        lfdPoint.pose.pose.position.x = self.pathInfo[lane][1].poses[lfdPointidx].pose.position.x
        lfdPoint.pose.pose.position.y = self.pathInfo[lane][1].poses[lfdPointidx].pose.position.y
        self.lfdPub.publish(lfdPoint)
        
        return steering

class LineChanger:
    """
    Decision line change or not.
    """
    def __init__(self, globalPathIn, globalPathOut)
        self.temp_velocity = 0
        self.lane_changing_flag = False
        self.stop_line_timer = rospy.Time.now().secs
        
        self.globalPathIn = globalPathIn
        self.globalPathOut = globalPathOut
        
    
    def line_change_check(self, path_index, distance, target_velocity, lidar_front, lidar_side_front, lidar_side_back):
        is_change = False
        temp_velocity = target_velocity
        if self.lane_changing_flag:     # 차선변경 중
            temp_velocity, is_done = self.lane_changing(target_velocity, distance)
            if is_done:                 # 차선변경 완료
                self.lane_changing_flag = False
                print("line change done")

        # Minimum velocity change
        if path_index in [0, 1, 3, 8]:  #추월 가능구간 확인 -> 2번구간 체크
            if(target_velocity < 13 and 25 > lidar_front > 5 and lidar_side_front > lidar_front+5 and lidar_side_back <=-15):  # 라인 변경 최소 거리 및 옆차선 차량 존재 x
                self.time_recorder()    #시간 기록 
                if (self.line_change_flag and rospy.Time.now().secs - self.stop_line_timer > 1.0): # 일정 정지 시간   
                    print("min_v line change start")
                    self.line_change(target_velocity)
                    is_change = True
            else:
                self.line_change_init()
                
        return is_change, temp_velocity

    def time_recorder(self):
        if self.line_change_flag == False:
            self.stop_line_timer = rospy.Time.now().secs
            self.line_change_flag = True
        
    def line_change(self, target_velocity):
        if not self.lane_changing_flag:
            self.line_change_init()
            self.temp_velocity = target_velocity
            print("temp vel", self.temp_velocity)
        self.lane_changing_flag = True

    def lane_changing(self, target_velocity, distance):
        # print "target",self.target_velocity, "temp",self.temp_velocity, "bf ctrl",self.ctrl_msg.velocity
        done_flag = False
        if target_velocity > self.temp_velocity:
            target_velocity = max(self.temp_velocity, 7)
        
        if self.lane == 1:
            dx = self.globalPathIn.poses[self.curWaypointIndxIn].pose.position.x - self.odom.pose.pose.position.x
            dy = self.globalPathIn.poses[self.curWaypointIndxIn].pose.position.y - self.odom.pose.pose.position.y
            dis = sqrt(dx*dx + dy*dy)
            if dis < 0.3:
                done_flag = True
                
        else:
            dx = self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.x - self.odom.pose.pose.position.x
            dy = self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.y - self.odom.pose.pose.position.y
            dis = sqrt(dx*dx + dy*dy)
            if dis < 0.3:
                return True
            else:
                return False
        
        return done_flag, target_velocity

    def line_change_init(self):
        self.line_change_flag = False
        # self.lane_changing_flag = False
    
class MergeChecker:
    """
    If merge section or not
    """
    def __init__(self):
        self.velocity_scale_side = 1.0
        self.moving_checker_flag = False
        self.merge_car_checker_flag = False
    
    def merge_check(self, curr_path_idx, lidar_side_front, lidar_side_back):
        """
        병합의 경우 차선 변경은 고려하지 않음.
        mergeChecker의 경우 Control 함수 내부에서 속도 계획의 일부로 사용됨. 
        병합 전 (8~0m) 주행에 대한 판단. 시작지점은 반복적인 주행으로 판단. 
        """
                  
        # 병합 전 8m~0m 지점에서 판단 (위치는 변경 가능)
        if curr_path_idx == 4: #병합 구간 확인
            
            if 0 < lidar_side_front < 15 or 0 > lidar_side_back > -15 :

                if self.merge_car_checker_flag == False:
                    self.merge(lidar_side_front, lidar_side_back)

            else:
                self.merge_init()

        return self.velocity_scale_side

    def merge_init(self):
        self.moving_checker_flag = False
        self.merge_car_checker_flag = False
        self.velocity_scale_side = 1.0

    def merge(self, lidar_side_front, lidar_side_back):
        # 합류 지점 직전에 내옆에 차가 존재 할 경우
        if (0 < lidar_side_front < 3 or -3 < lidar_side_back < 0):
            self.velocity_scale_side = 0.5
        
        # 있다 사라질 경우
        else:
            self.merge_init()
                
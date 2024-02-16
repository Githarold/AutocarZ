#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

import rospy
import rospkg
from math import cos, sin, sqrt, atan2, pi, tan, log, floor
import tf
import time

#ROS msg import
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from autocarz.msg import Erp42State, CtrlCmd, WaypointInfo

class planner:

    # package
    pkg="autocarz"
    rospack=rospkg.RosPack()
    pkgPath=rospack.get_path(pkg)
    pathInFile=rospy.get_param("pathIn")
    pathOutFile=rospy.get_param("pathOut")

    # path
    globalPathIn=Path()
    numGlobalPathIn=2224
    globalPathOut=Path()
    numGlobalPathOut=2255
    lane=1            # 1:In, 2:Out2
    out_path_index=0
    in_path_index=0

    # state
    odom=Odometry()        # position, orientation
    pastPos=[0.0, 0.0]
    curWaypointIndxIn=0
    curWaypointIndxOut=0
    waypointInfo=WaypointInfo()
    velocity=0.0            # km/h
    heading=0.0             # rad
    steering=0.0            # deg
    beforeHeading =0.0

    # pure-pursuit
    carLength=1.3
    
    # control
    lidar_d_front=30.0       
    lidar_d_side_front=30.0
    lidar_d_side_back=-15.0
    target_velocity=0.0     # km/h
    max_velocity=18.0       # km/h
    velocity_scale_side=1.0 # when side object is detected
    velocity_scale_path=1.0 # path별 속도 다른 것 scale로 적용

    # sync
    checkState=False
    checkGPS=False
    checkHeading=False

    # msg
    ctrl_msg=CtrlCmd()

    # merge
    moving_checker_flag = False 
    merge_car_checker_flag = False
    stop_lane_change_checker_flag = False
    min_v_lane_change_checker_flag = False
    lane_changing_flag = False

    moving_timer = 0
    stop_line_timer =0

    list_avg_heading = []

    #gps_safety
    gps_status = 0
    gps_covariance = 0.0


    # for test-->need to remove
    # t1=0.0
    # t2=0.0
    # test_trigger=1
    # f_b = open(pkgPath+"/brake.txt","w")
    # f_now_v = open(pkgPath+"/now_velocity.txt","w")
    # f_target_v =open(pkgPath+"/target_velocity.txt","w")

    def __init__(self):
        rospy.init_node("planner_vector", anonymous=False)

        # Subscriber
        rospy.Subscriber("/erp42/state", Erp42State, self.getState) 
        rospy.Subscriber("/ublox/fix", NavSatFix, self.getGPS) ################################ need to change
        # rospy.Subscriber("/distance_front", Float32, self.getLidar_f) #for test-->need to remove
        # rospy.Subscriber("/distance_side_front", Float32, self.getLidar_s_f) #for test-->need to remove
        # rospy.Subscriber("/distance_side_back", Float32, self.getLidar_s_b) #for test-->need to remove

        #debug log 
        self.distance_log_in=rospy.Publisher('distance_error_in',Float32,queue_size=1)
        self.distance_log_out=rospy.Publisher('distance_error_out',Float32,queue_size=1)
        self.last_gps_in = rospy.Time.now()


        # Publisher
        odomPub=rospy.Publisher('/odom', Odometry, queue_size=1)
        self.lfdPub=rospy.Publisher('/lfd', Odometry, queue_size=1)
        waypointInfoPub=rospy.Publisher('/waypointInfo', WaypointInfo, queue_size=1)
        ctrlPub = rospy.Publisher('/ctrlCmd', CtrlCmd, queue_size=1)
        pathInPub=rospy.Publisher('/pathIn', Path, queue_size=1)
        pathOutPub=rospy.Publisher('/pathOut', Path, queue_size=1)
        
        self.globalPathIn=self.readPath(self.pathInFile)
        self.globalPathOut=self.readPath(self.pathOutFile)
        #rospy variable
        self.moving_timer = rospy.Time.now().secs
        self.stop_line_timer = rospy.Time.now().secs

        while not rospy.is_shutdown():
            if self.checkState and self.checkGPS and self.checkHeading:
                self.localization(1)
                self.localization(2)
                self.makeWaypointInfo()
                waypointInfoPub.publish(self.waypointInfo)

                odomPub.publish(self.odom)
                self.purePursuit(self.lane)
                self.control()

                # self.ctrl_msg.brake = 0
                # self.ctrl_msg.longlCmdType = 2
                # self.ctrl_msg.velocity = 5

                # self.mergeChecker(self.lane)
                # self.line_change_checker(self.lane)
                

                ctrlPub.publish(self.ctrl_msg) # Send calculated 'ctrl_msg' to 'serialControl.py'

                # for visualization
                self.broadcast()
                pathInPub.publish(self.globalPathIn)
                pathOutPub.publish(self.globalPathOut)
        
        self.logFile.close()
        # self.f_b.close() #for test-->need to remove
        # self.f_now_v.close() #for test-->need to remove
        # self.f_target_v.close() #for test-->need to remove

    def gps_status_checker(self):
        if rospy.Time.now() -  self.last_gps_in > rospy.Duration(0.5):
            rospy.loginfo( "GPS TOPIC TIME ERROR")
            return False
        if self.gps_status != 2:
            line_index = 0
            if self.lane ==1:
                line_index =self.in_path_index
            else:
                line_index = self.out_path_index
            
            for i in [0,1,3,5,8]:
                if i == line_index:
                    if self.gps_covariance < 0.01:
                        rospy.loginfo( "GPS STATUS 0 GO "+str(self.gps_covariance))
                        return True
            rospy.loginfo( "GPS STATUS 0 STOP "+str(self.gps_covariance))
            return False
        #rospy.loginfo("GPS OKAY")
        return True

    def readPath(self, fileName):
        openFile=open(fileName, "r")

        globalPath=Path()
        globalPath.header.frame_id="/map"
        lines=openFile.readlines()
        for line in lines:
            data=line.split()
            pose=PoseStamped()
            pose.pose.position.x=float(data[0])
            pose.pose.position.y=float(data[1])
            pose.pose.position.z=0.0
            pose.pose.orientation.x=0.0
            pose.pose.orientation.y=0.0
            pose.pose.orientation.z=0.0
            pose.pose.orientation.w=float(data[2])
            globalPath.poses.append(pose)

        openFile.close()

        return globalPath

    def broadcast(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z),
                        (self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z),
                        rospy.Time.now(),
                        "base_link",
                        "map")

    def localization(self, lane):
        curX=self.odom.pose.pose.position.x
        curY=self.odom.pose.pose.position.y
        temp =Float32()

        minDist=float("inf")

        if lane==1:
            for i in range(len(self.globalPathIn.poses)):
                dx=curX-self.globalPathIn.poses[i].pose.position.x
                dy=curY-self.globalPathIn.poses[i].pose.position.y
                dist=sqrt(dx*dx+dy*dy)
                if dist<minDist:
                    self.curWaypointIndxIn=i
                    minDist=dist
            temp.data = minDist        
            self.distance_log_in.publish(temp)
        else:
            for i in range(len(self.globalPathOut.poses)):
                dx=curX-self.globalPathOut.poses[i].pose.position.x
                dy=curY-self.globalPathOut.poses[i].pose.position.y
                dist=sqrt(dx*dx+dy*dy)
                if dist<minDist:
                    self.curWaypointIndxOut=i
                    minDist=dist
            temp.data = minDist        
            self.distance_log_out.publish(temp)
        # print(curX,self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.x, curY,self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.y)

        
    def makeWaypointInfo(self):
        self.waypointInfo.lane=self.lane
        self.waypointInfo.indxIn=self.curWaypointIndxIn
        self.waypointInfo.indxOut=self.curWaypointIndxOut

    def purePursuit(self, lane):
        curX=self.odom.pose.pose.position.x
        curY=self.odom.pose.pose.position.y
        curVelocity=self.velocity/3.6   # m/s
        curHeading=self.heading
        bound=100+50
        # pp=curVelocity*5.0/5.45
        # lfdMin=5
        # lfdMax=30*pp
        
        inner_index = self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w
        outer_index = self.globalPathOut.poses[self.curWaypointIndxOut].pose.orientation.w

        # lfd=(curVelocity*3.6)*0.237+3.29
        # lfd=5
        #1104
        # curVelocity = 18/3.6
        # print(inner_index)
        lfdPointidx = 1
        # if lane==1:
        #     if(inner_index == 0 or inner_index == 1):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.3841+5.85
        #     elif(inner_index == 2):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.675+2.875
        #     elif(inner_index == 3):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.3841+5.85
        #     elif(inner_index == 4):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.675+2.875
        #     elif(inner_index == 5):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.3841+5.85
        #     elif(inner_index == 6):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.675+2.875
        #     elif(inner_index == 7):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*1.035+0.275
        #     elif(inner_index == 8):
        #         if(curVelocity<6/3.6):
        #             curVelocity=6/3.6
        #         lfd=(curVelocity)*0.3841+5.85
        #     else:
        #         print("inner_index error")
            #1104_in
            # lfd=2.0
            # print("lfd: ",lfd)
        ### pre lfd ###
        if lane==1:
            if(inner_index == 0 or inner_index == 7):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=7.5
            elif(inner_index == 1):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=7.7
            elif(inner_index == 2 or inner_index == 3):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=6.8
            elif(inner_index == 4):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=4.5
            elif(inner_index == 5 or inner_index == 6):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=5.5
            else:
                print("inner_index error")

            curWaypoint=self.curWaypointIndxIn
            if curWaypoint+bound>=self.numGlobalPathIn:
                localWaypoints=range(curWaypoint, self.numGlobalPathIn)+range(0,bound - (self.numGlobalPathIn-curWaypoint))
            else:
                localWaypoints=range(curWaypoint, curWaypoint+bound)

            for i in localWaypoints:
                dx=self.globalPathIn.poses[i].pose.position.x-curX
                dy=self.globalPathIn.poses[i].pose.position.y-curY
                rx=cos(curHeading)*dx+sin(curHeading)*dy
                ry=sin(curHeading)*dx-cos(curHeading)*dy
                # print("ry",ry)
                if rx>0.0:
                    dist=sqrt(rx*rx+ry*ry)
                    if dist>=lfd:
                        theta=atan2(ry, rx)
                        self.ctrl_msg.steering=-atan2(2*self.carLength*sin(theta), lfd)
                        lfdPointidx = i
                        break
        
        else:
            if(outer_index == 0 or outer_index == 1):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.3841+5.85
            elif(outer_index == 2):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.675+2.875
            elif(outer_index == 3):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.3841+5.85
            elif(outer_index == 4):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.675+2.875
            elif(outer_index == 5):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.3841+5.85
            elif(outer_index == 6):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.675+2.875
            elif(outer_index == 7):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*1.35-0.25
            elif(outer_index == 8):
                if(curVelocity<6/3.6):
                    curVelocity=6/3.6
                lfd=(curVelocity)*0.3841+5.85
            else:
                print("outer_index error")
            #1104
            # lfd=0.5
            # print("lfd: ",lfd)
            curWaypoint=self.curWaypointIndxOut
            if curWaypoint+bound>=self.numGlobalPathOut:
                localWaypoints=range(curWaypoint, self.numGlobalPathIn)+range(0,bound - (self.numGlobalPathIn-curWaypoint))
            else:
                localWaypoints=range(curWaypoint, curWaypoint+bound)

            for i in localWaypoints:
                dx=self.globalPathOut.poses[i].pose.position.x-curX
                dy=self.globalPathOut.poses[i].pose.position.y-curY
                rx=cos(curHeading)*dx+sin(curHeading)*dy
                ry=sin(curHeading)*dx-cos(curHeading)*dy
                # print("ry",ry)

                if rx>0.0:
                    dist=sqrt(rx*rx+ry*ry)
                    if dist>=lfd:
                        theta=atan2(ry, rx)
                        self.ctrl_msg.steering=-atan2(2*self.carLength*sin(theta), lfd)
                        lfdPointidx = i
                        break

               
        lfdPoint=Odometry()
        lfdPoint.header.frame_id="map"
        lfdPoint.pose.pose.position.z=0.0
        lfdPoint.pose.pose.orientation.x=0.0
        lfdPoint.pose.pose.orientation.y=0.0
        lfdPoint.pose.pose.orientation.z=0.0
        lfdPoint.pose.pose.orientation.w=1.0

        if self.lane==1:
            lfdPoint.pose.pose.position.x=self.globalPathIn.poses[lfdPointidx].pose.position.x
            lfdPoint.pose.pose.position.y=self.globalPathIn.poses[lfdPointidx].pose.position.y
        else:
            lfdPoint.pose.pose.position.x=self.globalPathOut.poses[lfdPointidx].pose.position.x
            lfdPoint.pose.pose.position.y=self.globalPathOut.poses[lfdPointidx].pose.position.y
        
        self.lfdPub.publish(lfdPoint)
        
        # print(curWaypoint, lfd, self.ctrl_msg.steering*180/pi, rx, ry)

    def control(self):

        inner_index = self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w
        outer_index = self.globalPathOut.poses[self.curWaypointIndxOut].pose.orientation.w
        self.out_path_index = outer_index
        self.in_path_index = inner_index

        add_speed = 2.0
        ref_velocity = 18.0+add_speed
        ###################velocity_scale_side#####################
        # if 0<self.lidar_d_side_front<7 and inner_index != 5 and outer_index != 5:
        #     self.velocity_scale_side=2.0/3.0
        # else:
        #     self.velocity_scale_side=1.0
        # #######only for 오르막길
        if inner_index==0 or outer_index==0:
            self.velocity_scale_side=1.0

        ###################velocity_scale_path#####################
        # self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w=1 ## for test-->need to remove
        # self.globalPathOut.poses[self.curWaypointIndxOut].pose.orientation.w=1 ## for test-->need to remove

        # if self.lane ==1: #in
        #     if inner_index==0:  # 18
        #         self.velocity_scale_path=(18.0+add_speed)/ref_velocity
        #     elif inner_index==1:    # 16.2
        #         self.velocity_scale_path=(16.0+add_speed)/ref_velocity    
        #     elif inner_index==2:    # 12.6    
        #         self.velocity_scale_path=14.0/ref_velocity
        #     elif inner_index==3:    # 18
        #         self.velocity_scale_path=(18.0+add_speed)/ref_velocity
        #     elif inner_index==4:    # 12.6
        #         self.velocity_scale_path=14.0/ref_velocity
        #     elif inner_index==5:    # 18
        #         self.velocity_scale_path=(18.0+add_speed)/ref_velocity
        #     elif inner_index==6:    # 12.6
        #         self.velocity_scale_path=14.0/ref_velocity
        #     elif inner_index==7:    # 12.6
        #         self.velocity_scale_path=14.0/ref_velocity
        #     elif inner_index==8:    # 7.2
        #         self.velocity_scale_path=8.0/ref_velocity

        #### pre vel ####
        if self.lane ==1: #in
            if inner_index==0:
                self.velocity_scale_path=18.0/18.0
            elif inner_index==1:
                self.velocity_scale_path=18.0/18.0
            elif inner_index==2:
                self.velocity_scale_path=16.0/18.0
            elif inner_index==3:
                self.velocity_scale_path=16.0/18.0
            elif inner_index==4:
                self.velocity_scale_path=16.0/18.0
            elif inner_index==5 or inner_index==6:
                self.velocity_scale_path=14.0/18.0
            elif inner_index==7:
                self.velocity_scale_path=8.0/18.0
            else:
                print("pathIn_index_error")

        if self.lane ==2: #out
            if outer_index==0:
                self.velocity_scale_path=(18.0+add_speed)/ref_velocity
            elif outer_index==1:
                self.velocity_scale_path=(16.0+add_speed)/ref_velocity    
            elif outer_index==2:
                self.velocity_scale_path=14.0/ref_velocity
            elif outer_index==3:
                self.velocity_scale_path=(18.0+add_speed)/ref_velocity
            elif outer_index==4:
                self.velocity_scale_path=14.0/ref_velocity
            elif outer_index==5:
                self.velocity_scale_path=(18.0+add_speed)/ref_velocity
            elif outer_index==6:
                self.velocity_scale_path=14.0/ref_velocity
            elif outer_index==7:
                self.velocity_scale_path=14.0/ref_velocity
            elif outer_index==8:
                self.velocity_scale_path=8.0/ref_velocity
            else:
                print("pathOut_index_error")
        
        min_v=6.0*self.velocity_scale_side
        if not self.lane_changing_flag:
            max_v=18.0*self.velocity_scale_side*self.velocity_scale_path
        else:
            max_v=6

        min_d=7.0
        max_d=25.0

        criteria_d= (min_d + max_d) / 2.0
        criteria_v=(min_v + (max_v - min_v)*(criteria_d-min_d)/(max_d-min_d)) * self.velocity_scale_side * self.velocity_scale_path

        p_gain=(max_v-min_v)/(max_d-min_d)
        i_gain=0.1
        d_gain=0.0
        controlTime=0.0333 #30Hz
        prev_error=0.0
        i_control=0.0
        
        self.ctrl_msg.brake = 0

        # Velocity(pid control)
        if self.lidar_d_front < min_d:
            self.target_velocity = 0
            self.ctrl_msg.brake = 100

        elif min_d <= self.lidar_d_front < max_d:
            error = self.lidar_d_front - criteria_d

            p_control=p_gain*error
            i_control+=i_gain*error*controlTime
            d_control=d_gain*(error-prev_error)/controlTime

            output=p_control+i_control+d_control
            prev_error=error

            result_v=criteria_v + output

            if (0 < result_v <= max_v):
                self.target_velocity = result_v
            elif result_v <= 0:
                self.target_velocity=0  
            else:
                self.target_velocity=max_v
            
       
        else:
            self.target_velocity = max_v
        
        self.ctrl_msg.velocity = self.target_velocity

        ### control vel for pre ###
        self.ctrl_msg.velocity = 18.0*self.velocity_scale_path
        

        if( not self.gps_status_checker()):
            self.ctrl_msg.longlCmdType = 2
            self.ctrl_msg.velocity = 0.0
            self.ctrl_msg.brake = 100

        # Brake(Pesudo ABS) --> need to test
        # self.ctrl_msg.longlCmdType = 2

        # if self.target_velocity >= min_v:
        #     if((self.target_velocity - self.velocity) + 2) < 0 and ((self.target_velocity-self.velocity) + 2) > -7000:
        #         self.ctrl_msg.brake = 6*(self.velocity-self.target_velocity) + 2*self.velocity + 30
        #     else:
        #         self.ctrl_msg.brake = 0
        # else:
        #     self.ctrl_msg.brake = 100

        # if self.ctrl_msg.brake > 100:
        #     self.ctrl_msg.brake = 100

        ###################just print#######################
        # print("min_v",min_v)
        # print("self.target_velocity",self.target_velocity)
        # print("velocity_target",self.ctrl_msg.velocity)
        # print("velocity_now",self.velocity)
        # print("brake",self.ctrl_msg.brake)
        #####################################################

        ############max_v-->min_v 감속시간 테스트###############
        # if self.target_velocity==14:
        #     self.t1=time.time()
        # if 5.98<self.target_velocity<6 and self.velocity<7.7:
        #     self.t2=time.time()
        # if self.t2!=0.0 and self.test_trigger==1:
        #     print(self.t2-self.t1)
        #     self.test_trigger=0
        #####################################################

        ####################for brake test###################
        # self.f_b.write(str(self.ctrl_msg.brake))
        # self.f_b.write('\n')

        # self.f_now_v.write(str(self.ctrl_msg.velocity))
        # self.f_now_v.write('\n')

        # self.f_target_v.write(str(self.target_velocity))
        # self.f_target_v.write('\n')
        ######################################################

    # callback function
    def getState(self, data):
        velocity = data.velocity # data.velocity 0~20/3.6 m/s  --> self.velocity 0~20 km/h, self.target_velocity 0~20 km/h
        
        if velocity > 100 or velocity<0:
            velocity = 0.0

        self.velocity= velocity
        self.steering=data.steering

        self.checkState=True

    def latlong2xy(self,lat, long):
        RE = 6371.00877 # 지구 반경(km)
        GRID = 0.0000005   # 격자 간격(km), 0.5mm단위 
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

    def getGPS(self, data):
        gpsX, gpsY=self.latlong2xy(data.latitude, data.longitude)
        self.odom.header.frame_id="map"
        self.odom.pose.pose.position.x=gpsX
        self.odom.pose.pose.position.y=gpsY
        self.odom.pose.pose.position.z=0.0   
        self.last_gps_in = rospy.Time.now()
        self.gps_status = data.status.status
        self.gps_covariance = data.position_covariance[0]
        # print(gpsX, gpsY)  
        if self.checkGPS and 1.0<self.velocity and self.velocity<50.0:
            dx=gpsX-self.pastPos[0]
            dy=gpsY-self.pastPos[1]
            dist=sqrt(dx**2+dy**2)
            
                           
            # and (not self.checkHeading or (abs(self.beforeHeading - heading) < pi/6))
            if dist>0.1  :
                # self.list_avg_heading.append(atan2(dy,dx))
                # avg = 0
                # if len(self.list_avg_heading)>4:
                #     self.list_avg_heading.pop(0)
                #     for i in self.list_avg_heading:
                #         avg += i
                #     self.heading = avg / len(self.list_avg_heading) 
                      
                self.heading=atan2(dy,dx)

                self.odom.pose.pose.orientation.x=0.0
                self.odom.pose.pose.orientation.y=0.0
                self.odom.pose.pose.orientation.z=sin(self.heading*0.5)
                self.odom.pose.pose.orientation.w=cos(self.heading*0.5)
                self.checkGPS=False
                self.checkHeading=True
                # self.beforeHeading = heading

        if not self.checkGPS:
            self.pastPos=[gpsX, gpsY]
            self.checkGPS=True

        if not self.checkHeading:
            print("No heading")
            

    def getLidar_f(self, data):
        self.lidar_d_front=data.data

    def getLidar_s_f(self, data):
        self.lidar_d_side_front=data.data

    def getLidar_s_b(self, data):
        self.lidar_d_side_back=data.data

    def mergeChecker(self,lane):
        #merge
        # 병합의 경우 차선 변경은 고려하지 않음.
        # mergeChecker의 경우 Control 함수 내부에서 속도 계획의 일부로 사용됨. 
        # 병합 전 (8~0m) 주행에 대한 판단. 시작지점은 반복적인 주행으로 판단. 

        if lane ==1 : #  In
      
                # 병합전 8m~0m 지점에서 판단 (위치는 변경 가능)
            if self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w==4: #병합 구간 확인
                
                if  0< self.lidar_d_side_front < 15 or 0> self.lidar_d_side_back > -15 :
                    #print("merge course")
                    if self.merge_car_checker_flag == False:
                        self.merge()

                else:
                    self.merge_init()


        else : # out 
                # 병합전 8m~0m 지점에서 2차 판단 (위치는 변경 가능)
            if self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w==4: #병합 구간 확인
                #print("merge course")
                if 0< self.lidar_d_side_front < 15 or 0 > self.lidar_d_side_back > -15:
                   
                    if self.merge_car_checker_flag == False:
                        self.merge()
            
                else:
                    self.merge_init()

        
        
    def merge_init(self):
        self.moving_checker_flag = False
        self.merge_car_checker_flag = False

            
    def merge(self):
        
        #합류 지점 직전에 내옆에 차가 존재 할 경우 
        if(0< self.lidar_d_side_front < 3 or -3 < self.lidar_d_side_back <0):
            print("merge stop")
            self.ctrl_msg.brake = 100
            self.ctrl_msg.velocity = 0
            #시간 기록
            if self.moving_checker_flag == False:
                self.moving_timer = rospy.Time.now().secs
                self.moving_checker_flag = True

            print(rospy.Time.now().secs - self.moving_timer)
            if (rospy.Time.now().secs - self.moving_timer > 5):
                self.merge_car_checker_flag = True
                print("merge_go\n\n\n\n\n\n\n\n")
                self.ctrl_msg.brake = 0

                #차가 멈춰있을 경우 체크 시간
        #있다 사라질 경우 
        else:
            self.merge_init()
                


    def line_change_checker(self,lane):
        if self.lane_changing_flag: # 차선변경 중
            if self.lane_changing(): #  차선변경 완료
                self.lane_changing_flag = False

    
        if lane == 1:
            #stop line change
            index =self.globalPathIn.poses[self.curWaypointIndxIn].pose.orientation.w
            if index==2: #추월 가능구간 확인
                if(self.velocity < 1 and 15>self.lidar_d_front > 3 and self.lidar_d_side_front >12 and self.lidar_d_side_back <=-15):  # 라인 변경 최소 거리 및 옆차선 차량 존재 x

                    print("chlin")
                    self.time_recorder()  #시간 기록 
                    if (self.line_change_flag ==True and rospy.Time.now().secs - self.stop_line_timer > 3): # 일정 정지 시간   
                        print("min_v line change")           
                        self.line_change(lane)
                else:
                    self.line_change_init()


            #minimum velocity change
            if index==1: #추월 가능구간 확인
                if(self.velocity < 7 and 15 > self.lidar_d_front > 3 and self.lidar_d_side_front >12 and self.lidar_d_side_back <=-15):  # 라인 변경 최소 거리 및 옆차선 차량 존재 x
                    print("chlin")
                    self.time_recorder()  #시간 기록 
                    if (self.line_change_flag ==True and rospy.Time.now().secs - self.stop_line_timer > 0): # 일정 정지 시간   
                        print("min_v line change")           
                        self.line_change(lane)
                else:
                    self.line_change_init()
                
        elif lane == 2:        
            #stop line change
            index = self.globalPathOut.poses[self.curWaypointIndxOut].pose.orientation.w
            if index==2: #추월 가능구간 확인
                if(self.velocity < 1 and 15 >self.lidar_d_front > 3 and self.lidar_d_side_front >12 and self.lidar_d_side_back <=-15):  # 라인 변경 최소 거리 및 옆차선 차량 존재 x
                    
                    self.time_recorder()     #시간 기록 
                    
                    if (self.line_change_flag ==True and rospy.Time.now().secs - self.stop_line_timer > 3): # 일정 정지 시간   
                        print("min_v line change")           
                        self.line_change(lane)                    
                else:
                    self.line_change_init()

            #minimum velocity change
            if index == 1: #추월 가능구간 확인
                
                if(self.velocity < 7 and 15 >self.lidar_d_front > 3 and self.lidar_d_side_front >12 and self.lidar_d_side_back <=-15):  # 라인 변경 최소 거리 및 옆차선 차량 존재 x
                    
                    self.time_recorder()      #시간 기록

                    if (self.line_change_flag ==True and rospy.Time.now().secs - self.stop_line_timer > 3): # 일정 정지 시간   
                        print("min_v line change")           
                        self.line_change(lane)    
                
                else:
                    self.line_change_init()
                
           
                    

    def time_recorder(self):
        
        if self.line_change_flag == False:
            self.stop_line_timer = rospy.Time.now().secs
            self.line_change_flag = True 
        
        
    def line_change(self,lane): 
        if self.lane_changing_flag == False:
            if lane == 1 :
                self.lane = 2
            elif lane == 2:
                self.lane = 1 
            self.line_change_init()
        self.lane_changing_flag = True


    def lane_changing(self):
        
        
        if self.lane == 1:
            dx = self.globalPathIn.poses[self.curWaypointIndxIn].pose.position.x -self.odom.pose.pose.position.x
            dy = self.globalPathIn.poses[self.curWaypointIndxIn].pose.position.y -self.odom.pose.pose.position.y
            dis = sqrt(dx*dx + dy*dy)
            print("lane_chaning")
            if dis < 0.3:
                return True
            else:
                return False

        else:
            dx = self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.x -self.odom.pose.pose.position.x
            dy = self.globalPathOut.poses[self.curWaypointIndxOut].pose.position.y -self.odom.pose.pose.position.y
            dis = sqrt(dx*dx + dy*dy)
            print("lane_chaning")        
            if dis < 0.3:
                return True
            else:
                return False
      


    def line_change_init(self):
        self.lanechange_timer_flag = False
        self.line_change_flag = False
        self.lane_changing_flag = False




if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_data import LidarSensor, GPS, VehicleState
from data_processing import LocalizationProcessor
from decision_making import PathPlanner, SpeedDecision, SteeringDecision, LineChanger, MergeChecker
from vehicle_control import VehicleControler
from utilities import Logger, ConfigManager
from autocarz.msg import CtrlCmd

class AutonomousVehicleController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("planner_vector", anonymous=False)
        
        # Path file 불러오기
        self.lane = 1
        numGlobalPathIn = 2223
        numGloablPathOut = 2255

        # 모듈 초기화
        self.config_manager = ConfigManager()
        self.localization_processor = LocalizationProcessor()
        
        # Global path 읽기
        pathInFile = self.config_manager.get_path('in')
        pathOutFile = self.config_manager.get_path('out')
        self.globalPathIn = self.localization_processor.readPath(pathInFile)
        self.globalPathOut = self.localization_processor.readPath(pathOutFile)
        
        # 모듈 초기화
        self.logger = Logger()              # 로깅 필요할 때만
        self.gps_sensor = GPS()
        self.lidar_sensor = LidarSensor()
        self.vehicle_state = VehicleState()
        self.path_planner = PathPlanner(self.globalPathIn, self.globalPathOut)
        self.speed_decision = SpeedDecision()
        self.steering_decision = SteeringDecision(numGlobalPathIn, numGloablPathOut, self.globalPathIn, self.globalPathOut)
        self.line_changer = LineChanger()
        self.merge_checker = MergeChecker()
        self.vehicle_control = VehicleControler()

    def run(self):
        target_velocity = 0
        while not rospy.is_shutdown():
            
            # 1. 센서 데이터 수집
            odom, heading, gps_status, gps_covariance, checkGPS, checkHeading, last_gps_in = self.gps_sensor.gps_data()
            velocity, _, checkERP = self.vehicle_state.vehicle_data()   # Steering 현재 안씀

            if checkGPS and checkHeading and checkERP:
                
                lidar_front, lidar_side_front, lidar_side_back = self.lidar_sensor.lidar_data(self.lane)
                
                # 2. 위치 파악 및 근방 인덱스와 오차 Pub
                curWaypointIndxIn, curWaypointIndxOut, distanceIn, distanceOut = self.path_planner.localization(odom, self.lane)
                in_path_idx = self.globalPathIn.poses[curWaypointIndxIn].pose.orientation.w
                out_path_idx = self.globalPathOut.poses[curWaypointIndxOut].pose.orientation.w
                curr_path_idx, distance = in_path_idx, distanceIn if self.lane == 1 else out_path_idx, distanceOut
                
                # 3. GPS 데이터가 제대로 들어오는지 확인
                if self.speed_decision.isGPSerror(gps_status, gps_covariance, last_gps_in, curr_path_idx):
                    continue
                
                # 4. 병합구간 확인 및 차선 변경
                velocity_scale_side = self.merge_checker.merge_check(curr_path_idx, lidar_side_front, lidar_side_back)
                change, _ = self.line_changer.line_change_check(curr_path_idx, distance, target_velocity, lidar_front, lidar_side_front, lidar_side_back)
                self.lane = 1 if self.lane == 2 else 2 if change else self.lane
                
                # 5. 차량 제어 결정
                target_velocity, brake = self.speed_decision.control_speed(lidar_front, curr_path_idx, velocity_scale_side)
                steering_angle = self.steering_decision.purePursuit(odom, self.lane, velocity, heading, curr_path_idx, curWaypointIndxIn, curWaypointIndxOut)

                # 6. 차량 제어 명령 실행
                self.vehicle_control.control(target_velocity, brake, steering_angle)

if __name__ == '__main__':
    try:
        vehicle_controller = AutonomousVehicleController()
        vehicle_controller.run()
    except rospy.ROSInterruptException:
        pass

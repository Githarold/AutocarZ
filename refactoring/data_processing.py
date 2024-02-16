#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import sqrt, atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
 
class LocalizationProcessor:
    """
    Processes localization data to determine the vehicle's position and orientation.
    """
    def __init__(self):
        pass

    def readPath(self, fileName):
        """
        Read path data from a file.
        """
        path = Path()
        path.header.frame_id = "/map"

        with open(fileName, "r") as file:
            lines = file.readlines()
            for line in lines:
                data = line.split()
                pose = PoseStamped()
                pose.pose.position.x = float(data[0])
                pose.pose.position.y = float(data[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = float(data[2])
                path.poses.append(pose)

        return path

class PathCalculator:
    """
    Handles calculations related to the path.
    """
    def __init__(self):
        pass

    def calculate_closest_waypoint(self, path, current_position):
        """
        Find the closest waypoint to the current position.
        """
        closest_index = 0
        min_distance = float('inf')
        for i, pose in enumerate(path.poses):
            dx = current_position.x - pose.pose.position.x
            dy = current_position.y - pose.pose.position.y
            distance = sqrt(dx*dx + dy*dy)
            if distance < min_distance:
                closest_index = i
                min_distance = distance

        return closest_index

    def calculate_desired_heading(self, current_position, target_position):
        """
        Calculate the desired heading to reach the target position.
        """
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
        heading = atan2(dy, dx)
        return heading

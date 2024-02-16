#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Logger:
    """
    A simple logging utility for ROS.
    """
    def __init__(self):
        pass

    def log(self, message, messageType='info'):
        """
        Log a message using ROS logging system.
        """
        if messageType == 'info':
            rospy.loginfo(message)
        elif messageType == 'warn':
            rospy.logwarn(message)
        elif messageType == 'error':
            rospy.logerr(message)
        # Additional message types can be added as needed

class ConfigManager:
    """
    Manages configuration parameters retrieved from ROS parameters.
    """
    def __init__(self):
        # Retrieve package path and ROS parameters
        self.pkg = "autocarz"
        self.rospack = rospkg.RosPack()
        self.pkgPath = self.rospack.get_path(self.pkg)
        self.pathInFile = rospy.get_param("pathIn")
        self.pathOutFile = rospy.get_param("pathOut")

    def get_path(self, pathType):
        """
        Return file paths for 'in' and 'out' path types.
        """
        if pathType == 'in':
            return self.readPath(self.pathInFile)
        elif pathType == 'out':
            return self.readPath(self.pathOutFile)
        
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
    
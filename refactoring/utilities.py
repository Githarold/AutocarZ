#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

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
            return self.pathInFile
        elif pathType == 'out':
            return self.pathOutFile
        
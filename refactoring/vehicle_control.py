#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from autocarz.msg import CtrlCmd

class VehicleControler:
    """
    Manages the steering of the vehicle.
    """    
    def __init__(self):
        self.ctrl_pub = rospy.Publisher('/ctrlCmd', CtrlCmd, queue_size=1)
        self.ctrl_msg = CtrlCmd()
        
    def control(self, speed, brake, steering):
        self.ctrl_msg.velocity = speed
        self.ctrl_msg.brake = brake
        self.ctrl_msg.steering = steering
        self.ctrl_pub.publish(self.ctrl_msg)
    
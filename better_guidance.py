# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 14:20:30 2015

@author: student
"""

import lcm
from lcmtypes import velocity_cmd_t
import math
import time

ANG_TOLERANCE = math.pi/180.0
DIST_TOLERANCE = 10.0

class Guidance:
    def __init__(self):
        self.next_point = (0, 0)
        self.state = -1
        self.lc = lcm.LCM()
        self.start_time = time.time()
    
    def start(self):
        self.state = 0
        
    def guide_and_command(self, pos):
        x = pos[0]
        y = pos[1]
        theta = pos[2]
        #heading = math.atan2(self.next_point[1] - y, x - self.next_point[0])
        heading = math.atan2(y - self.next_point[1], x - self.next_point[0])
        dist = math.sqrt(math.pow(self.next_point[0]-x, 2) + math.pow(self.next_point[1] - y,2))
                
        if self.state == 0:
            self.start_time = time.time()
            if math.fabs(heading - theta) < ANG_TOLERANCE:
                print "Reached heading"
                self.state = 2
                return
            else:
                print "Desired Heading:", math.degrees(heading)
                print "Actual Heading:", math.degrees(theta)
                command = velocity_cmd_t()
                command.Angle = math.degrees(heading - theta)
                print "Turning by:", command.Angle
                if command.Angle > 0:
                    command.AngSpeed = -25.0
                else:
                    command.AngSpeed = 25.0
                self.lc.publish("GS_VELOCITY_CMD", command.encode())
                self.state = 1
        
        if self.state == 1:
            if math.fabs(heading - theta) < ANG_TOLERANCE or (time.time() - self.start_time) > 5:
                print "Reached heading"
                self.state = 2
                return

        if self.state == 2:
            self.start_time = time.time()
            print "Dist:",dist
            if dist < DIST_TOLERANCE:
                print "Reached destination"
                self.state = -1
                return
            else:
                command = velocity_cmd_t()
                command.Distance = dist
                command.FwdSpeed = 80.0
                self.lc.publish("GS_VELOCITY_CMD", command.encode())
                self.state = 3
        
        if self.state == 3:
            if dist < DIST_TOLERANCE or (time.time() - self.start_time) > 5:
                self.state = -1
                return
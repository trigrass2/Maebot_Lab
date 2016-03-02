# PID.py
#
# skeleton code for University of Michigan ROB550 Botlab
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys, os, time,signal
import lcm
import math 
# import PID

from lcmtypes import maebot_diff_drive_t
from lcmtypes import maebot_motor_feedback_t
from lcmtypes import velocity_cmd_t
from lcmtypes import pid_init_t
#from lcmtypes import maebot_command_t


class PID():
    def __init__(self, kp, ki, kd):
        # Main Gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
            
        # Integral Terms
        self.iTerm = 0.0
        self.iTermMin = -0.2
        self.iTermMax = 0.2

        # Input (feedback signal)
        self.prevInput = 0.0
        self.input = 0.0

        # Reference Signals (signal and derivative!)
        self.dotsetpoint = 0.0          
        self.setpoint = 0.0

        # Output signal and limits
        self.output = 0.0
        self.outputMin = -0.4
        self.outputMax = 0.4

        # Update Rate
        self.updateRate = 0.1

        # Error Signals
        self.error = 0.0
        self.errordot = 0.0

    def Compute(self):
        # IMPLEMENT ME!
        # Different then last project! 
        # Now you have a speed reference also!
        # Also Update Rate can be gone here or in SetTunnings function
        # (it is your choice!) But change functions consistently

        Vel = (self.input - self.prevInput) / self.updateRate

        self.errordot = self.dotsetpoint - Vel

        ud = self.kd * self.errordot

        self.setpoint += self.dotsetpoint * self.updateRate

        self.error = self.setpoint - self.input

        up = self.kp * self.error

        self.iTerm += self.error * self.updateRate

        self.iTerm = min(max(self.iTerm, self.iTermMin), self.iTermMax)
        
        ui = self.ki * self.iTerm

        self.output = min(max(ui+ud+up,self.outputMin),self.outputMax)
        self.prevInput = self.input
 
    # Accessory Function to Change Gains
    # Update if you are not using updateRate in Compute()
    def SetTunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki 
        self.kd = kd

    def SetIntegralLimits(self, imin, imax):
        self.iTermMin = imin
        self.iTermMax = imax

    def SetOutputLimits(self, outmin, outmax):
        self.outputMin = outmin
        self.outputMax = outmax
                
    def SetUpdateRate(self, rateInSec):
        self.updateRate = rateInSec



class PIDController():
    def __init__(self):
     
        # Create Both PID
        #self.leftCtrl =  PID(0.0008,0.000,0.0004)
        #self.rightCtrl = PID(0.0008,0.000,0.0004)
        self.leftCtrl =  PID(0.0,0.0,0.0)
        self.rightCtrl = PID(0.0,0.0,0.0)
 
        # LCM Subscribe
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("MAEBOT_MOTOR_FEEDBACK", 
                                        self.motorFeedbackHandler)
        lcmCommandSub = self.lc.subscribe("GS_VELOCITY_CMD",
                                        self.motorCommandHandler)
        lcmComandSud = self.lc.subscribe("GS_PID_INIT",
                                        self.PIDInitHandler)
        signal.signal(signal.SIGINT, self.signal_handler)


        # Odometry 
        self.wheelDiameterMillimeters = 32.0  # diameter of wheels [mm]
        self.axleLengthMillimeters = 80.0     # separation of wheels [mm]    
        self.ticksPerRev = 16.0           # encoder tickers per motor revolution
        self.gearRatio = 30.0             # 30:1 gear ratio
        self.enc2mm = ((math.pi * self.wheelDiameterMillimeters) / 
                       (self.gearRatio * self.ticksPerRev)) 
                        # encoder ticks to distance [mm]

        # Initialization Variables
        self.InitialPosition = (0.0, 0.0)
        self.startDistance = (0,0)
        self.curDistance = (0,0)
        self.oldTime = 0.0
        self.startTime = 0.0

        self.init_pos = False

        self.mu0 = 1.0
        self.trim = 0.2

        #self.DesiredDistance = 0
        self.DesiredDistance = (0,0) #Left,Right
        self.DesiredAngle = 0
        self.AngularSpeed = 0

    # def publishMotorCmd(self):
    #     cmd = maebot_diff_drive_t()
    #     # IMPLEMENT ME
    #     distanceDifference = self.curDistance[0] - self.startDistance[0]
    #     if distanceDifference >= self.DesiredDistance:
    #         cmd.motor_left_speed = 0
    #         cmd.motor_right_speed = 0
    #         self.leftCtrl.setpoint = self.curDistance[0]
    #         self.rightCtrl.setpoint = self.curDistance[1]
    #         self.leftCtrl.dotsetpoint = 0
    #         self.rightCtrl.dotsetpoint = 0
    #     else:
    #         cmd.motor_left_speed = (self.leftCtrl.output + self.trim) * self.mu0
    #         cmd.motor_right_speed = (self.rightCtrl.output + self.trim) * self.mu0
    #         print "Left : %.2f , %.2f" % (cmd.motor_left_speed, cmd.motor_right_speed)
            
    #     self.lc.publish("MAEBOT_DIFF_DRIVE", cmd.encode())

    def publishMotorCmd(self):
        cmd = maebot_diff_drive_t()
        # IMPLEMENT ME
        distanceDifferenceLeft = abs(self.curDistance[0] - self.startDistance[0])
        distanceDifferenceRight = abs(self.curDistance[1] - self.startDistance[1])
        if distanceDifferenceLeft >= abs(self.DesiredDistance[0]) or distanceDifferenceRight >= abs(self.DesiredDistance[1]):
            cmd.motor_left_speed = 0
            self.leftCtrl.setpoint = self.curDistance[0]
            self.leftCtrl.dotsetpoint = 0
        
            cmd.motor_right_speed = 0
            self.rightCtrl.setpoint = self.curDistance[1]
            self.rightCtrl.dotsetpoint = 0

        else:
            lWheelDir = 1 #for setting left wheel trim value to negative or positive (backward/leftturn/rightturn)
            rWheelDir = 1 #for setting left wheel trim value to negative or positive (backward/leftturn/rightturn)
            if self.leftCtrl.dotsetpoint < 0 : 
                lWheelDir = -1
            if self.rightCtrl.dotsetpoint < 0 :
                rWheelDir = -1 
            cmd.motor_left_speed = (self.leftCtrl.output + self.trim * lWheelDir) * self.mu0
            cmd.motor_right_speed = (self.rightCtrl.output + self.trim * rWheelDir) * self.mu0

            #cmd.motor_left_speed = (self.leftCtrl.output + self.trim ) * self.mu0 
            #cmd.motor_right_speed = (self.rightCtrl.output + self.trim ) * self.mu0

            #cmd.motor_left_speed = min(max(cmd.motor_left_speed,-0.4),0.4) #added to limit output
            #cmd.motor_right_speed = min(max(cmd.motor_left_speed,-0.4),0.4) #added to limit output

            #print "Left PID: %.2f Left Total: %.2f Right PID: %.2f Right Total: %.2f" % (self.leftCtrl.output, cmd.motor_left_speed, self.rightCtrl.output, cmd.motor_right_speed)
          
        self.lc.publish("MAEBOT_DIFF_DRIVE", cmd.encode())


    def motorFeedbackHandler(self,channel,data):
        msg = maebot_motor_feedback_t.decode(data)
        #Save where we are now
        lDist = msg.encoder_left_ticks * self.enc2mm
        rDist = msg.encoder_right_ticks * self.enc2mm
        self.curDistance = (lDist, rDist)

        self.leftCtrl.input = lDist
        self.rightCtrl.input = rDist
        self.startTime = msg.utime_sama5 / 1000000.0

        if not self.init_pos:
            self.oldTime = self.startTime
            self.leftCtrl.setpoint = self.leftCtrl.input
            self.rightCtrl.setpoint = self.rightCtrl.input
            self.leftCtrl.prevInput = lDist #added by Kevin
            self.rightCtrl.prevInput = rDist #added by Kevin
            self.init_pos = True
        else:
            self.leftCtrl.updateRate = self.startTime - self.oldTime
            self.rightCtrl.updateRate = self.startTime - self.oldTime
            self.leftCtrl.Compute()
            self.rightCtrl.Compute()
        
        self.oldTime = self.startTime

    # def motorCommandHandler(self, channel, data):
    #     msg = velocity_cmd_t.decode(data)

    #     self.DesiredAngle = msg.Angle
    #     self.AngularSpeed = msg.AngSpeed

    #     self.rightCtrl.dotsetpoint = msg.FwdSpeed 
    #     self.leftCtrl.dotsetpoint = msg.FwdSpeed 

    #     self.DesiredDistance = msg.Distance 
    #     self.startDistance = self.curDistance 

    def motorCommandHandler(self, channel, data):
        msg = velocity_cmd_t.decode(data)
        print ("command distance, angle: (%f,%f)") % (msg.Distance, msg.Angle)
        self.AngularSpeed = msg.AngSpeed * math.pi/180.00 #convert to radians/sec

        self.rightCtrl.dotsetpoint = msg.FwdSpeed + ((self.axleLengthMillimeters/2.0) * self.AngularSpeed)
        self.leftCtrl.dotsetpoint = msg.FwdSpeed - ((self.axleLengthMillimeters/2.0) * self.AngularSpeed)

        self.DesiredAngle = msg.Angle * math.pi/180.00 #convert to radians

        self.DesiredDistance = (msg.Distance - (self.axleLengthMillimeters/2.0) * self.DesiredAngle,
                                msg.Distance + (self.axleLengthMillimeters/2.0) * self.DesiredAngle ) #left,right
        #print "DesiredDistance (Left, Right): %s" % (self.DesiredDistance,)
        #print "DotSetpoint (Left, Right): %f , %f " % (self.leftCtrl.dotsetpoint, self.rightCtrl.dotsetpoint)
        self.startDistance = self.curDistance     

        
    #     #STUFF

    def PIDInitHandler(self, channel, data):
        msg = pid_init_t.decode(data)

        self.rightCtrl.SetTunings(msg.kp, msg.ki, msg.kd)
        self.rightCtrl.iTerm = 0
        self.leftCtrl.SetTunings(msg.kp, msg.ki, msg.kd)
        self.leftCtrl.iTerm = 0


    
    def Controller(self):

        # For now give a fixed command here
        # Later code to get from groundstation should be used

        print "Starting loop"

        while(1):
            self.lc.handle()
            # IMPLEMENT ME
            # MAIN CONTROLLER
            self.publishMotorCmd()
       
    # Function to print 0 commands to morot when exiting with Ctrl+C 
    # No need to change 
    def signal_handler(self,signal, frame):
        print("Terminating!")
        for i in range(5):
            cmd = maebot_diff_drive_t()
            cmd.motor_right_speed = 0.0
            cmd.motor_left_speed = 0.0  
            self.lc.publish("MAEBOT_DIFF_DRIVE",cmd.encode())
        exit(1)

pid = PIDController()
pid.Controller()

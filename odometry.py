# odometry.py
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

import sys, os, time
import lcm
from math import *

from breezyslam.robots import WheeledRobot

from lcmtypes import maebot_motor_feedback_t
from lcmtypes import maebot_sensor_data_t
from lcmtypes import odo_pose_xyt_t
from lcmtypes import odo_dxdtheta_t

class Maebot(WheeledRobot):
  GYRO_THRESHOLD = 0.01
    
  def __init__(self):
    self.wheelDiameterMillimeters = 32.0     # diameter of wheels [mm]
    self.axleLengthMillimeters = 80.0        # separation of wheels [mm]  
    self.ticksPerRev = 16.0                  # encoder tickers per motor revolution
    self.gearRatio = 30.0                    # 30:1 gear ratio
    self.enc2mm = (pi * self.wheelDiameterMillimeters) / (self.gearRatio * self.ticksPerRev) # encoder ticks to distance [mm]
    print self.enc2mm
    self.prevEncPos = (0,0,0)           # store previous readings for odometry
    self.prevOdoPos = (0,0,0)           # store previous x [mm], y [mm], theta [rad]
    self.currEncPos = (0,0,0)           # current reading for odometry
    self.currOdoPos = (0,0,0)           # store odometry x [mm], y [mm], theta [rad]
    self.currOdoVel = (0,0,0)           # store current velocity dxy [mm], dtheta [rad], dt [s]
    self.prevGyroData = (0, 0)
    self.currGyroData = (0, 0)
    WheeledRobot.__init__(self, self.wheelDiameterMillimeters/2.0, self.axleLengthMillimeters/2.0)

    # LCM Initialization and Subscription
    self.lc = lcm.LCM()
    lcmMotorSub = self.lc.subscribe("MAEBOT_MOTOR_FEEDBACK", self.motorFeedbackHandler)
    lcmSensorSub = self.lc.subscribe("MAEBOT_SENSOR_DATA", self.sensorDataHandler)

    self.init_odo = False
    self.init_gyro = 0
    self.gyro_bias = [] # initializing gyro bias
    self.cali_time = 10 * 1000000 # calibration time in us
    self.init_time = 0

    self.c_l = 1.0058
    self.c_r = 0.9942
    self.e_b = 1.0403

  def calcVelocities(self):
    # IMPLEMENT ME
    # TASK: CALCULATE VELOCITIES FOR ODOMETRY
    # Update self.currOdoVel and self.prevOdoVel
    dL = (self.currEncPos[0] - self.prevEncPos[0]) * self.enc2mm * self.c_l
    dR = (self.currEncPos[1] - self.prevEncPos[1]) * self.enc2mm * self.c_r
    dt = self.currEncPos[2] - self.prevEncPos[2] #us
    #print self.currEncPos

    dt_gyro = (self.currGyroData[1] - self.prevGyroData[1]) #us

    theta_odo = (dR-dL)/self.axleLengthMillimeters #radians

    theta_gyro = (self.currGyroData[0]/131.0 * dt_gyro) * (pi/180.0)/1000000 #radians

    # Correction in gyro calculations
    # theta_gyro = (self.currGyroData[0] * (500 / 65536)) / 131.0 * (pi/180.0) / 1000000 #radians/us

    if (abs(theta_gyro - theta_odo) > Maebot.GYRO_THRESHOLD) : 
        dtheta = theta_gyro
        print "gyro"
    else:
        dtheta = theta_odo

    print abs(theta_gyro - theta_odo)

    #dxy = min(dL, dR) + abs(dL-dR)/2.0*sin(dtheta)
    dxy = (dR+dL)/2.0
    #vxy = (tan(theta)*self.axleLengthMillimeters/2.0 + dR) / dt #mm/us

    self.currOdoVel = (dxy, dtheta, dt)

  def getVelocities(self):
    # IMPLEMENT ME
    # TASK: RETURNS VELOCITY TUPLE
    # Return a tuple of (dxy [mm], dtheta [rad], dt [s])
    return self.currOdoVel # [mm], [rad], [s]

  def calcOdoPosition(self):
    # IMPLEMENT ME
    # TASK: CALCULATE POSITIONS
    # Update self.currOdoPos and self.prevOdoPos
    self.prevOdoPos = self.currOdoPos
    
    theta_global = self.prevOdoPos[2] + self.currOdoVel[1]
    x_global = self.prevOdoPos[0] + self.currOdoVel[0]*cos(theta_global)
    y_global = self.prevOdoPos[1] + self.currOdoVel[0]*sin(theta_global)
    

    self.currOdoPos = (x_global, y_global, theta_global)
    print self.currOdoPos

  def getOdoPositiotimen(self):
    # IMPLEMENT ME
    # TASK: RETURNS POSITION TUPLE
    # Return a tuple of (x [mm], y [mm], theta [rad])
    return self.currOdoPos # [mm], [rad], [s]

  def publishOdometry(self):
    # IMPLEMENT ME
    # TASK: PUBLISHES BOT_ODO_POSE MESSAGE
    msg = odo_pose_xyt_t()
    msg.utime = time.time()
    msg.xyt = self.currOdoPos

    self.lc.publish("ODOMETRY", msg.encode())

  def publishVelocities(self):
    # IMPLEMENT ME
    # TASK: PUBLISHES BOT_ODO_VEL MESSAGE
    msg = odo_dxdtheta_t()
    msg.utime = time.time()
    msg.dxy = self.currOdoVel[0]
    msg.dtheta = self.currOdoVel[1]
    msg.dt = self.currOdoVel[2]

    self.lc.publish("VELOCITY", msg.encode())

  def motorFeedbackHandler(self,channel,data):
    msg = maebot_motor_feedback_t.decode(data)
    # IMPLEMENT ME
    # TASK: PROCESS ENCODER DATA
    # get encoder positions and store them in robot,
    # update robots position and velocity estimate

    self.prevEncPos = self.currEncPos
    self.currEncPos = (msg.encoder_left_ticks, 
                        msg.encoder_right_ticks, 
                        msg.utime_sama5)

    if self.init_odo and self.init_gyro > 2:
        self.calcVelocities()
        self.calcOdoPosition()
    else:
        self.init_odo = True



  def sensorDataHandler(self,channel,data):
    msg = maebot_sensor_data_t.decode(data)
    # IMPLEMENT ME
    # TASK: PROCESS GYRO DATA

    self.prevGyroData = self.currGyroData
    self.currGyroData = (msg.gyro[2], msg.utime_sama5)

    if self.init_gyro <= 2:
       self.init_gyro += 1
    
    # Uncommment for gyro bias correction
    # if  self.init_gyro < 50 :
    #     self.gyro_bias.append(msg.gyro[2])
    #     self.init_gyro +=1
    # else :
    #     self.prevGyroData = self.currGyroData
    #     self.currGyroData = (msg.gyro[2] - (sum(gyro_bias)/self.init_gyro), msg.utime_sama5)



  def MainLoop(self):
    oldTime = time.time()
    frequency = 20;
    while(1):
      self.lc.handle()
      if(time.time()-oldTime > 1.0/frequency):
        self.publishOdometry()
        self.publishVelocities()
        oldTime = time.time()   


if __name__ == "__main__":
  
  robot = Maebot()
  robot.MainLoop()  

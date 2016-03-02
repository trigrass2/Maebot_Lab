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
    
  def __init__(self):
    self.wheelDiameterMillimeters = 32.0     # diameter of wheels [mm]
    self.axleLengthMillimeters = 80.0        # separation of wheels [mm]  
    self.ticksPerRev = 16.0                  # encoder tickers per motor revolution
    self.gearRatio = 30.0                    # 30:1 gear ratio
    self.enc2mm = (pi * self.wheelDiameterMillimeters) / (self.gearRatio * self.ticksPerRev) # encoder ticks to distance [mm]

    self.prevEncPos = (0,0,0)           # store previous readings for odometry
    self.prevOdoPos = (0,0,0)           # store previous x [mm], y [mm], theta [rad]
    self.currEncPos = (0,0,0)           # current reading for odometry
    self.currOdoPos = (0,0,0)           # store odometry x [mm], y [mm], theta [rad]
    self.currOdoVel = (0,0,0)           # store current velocity dxy [mm], dtheta [rad], dt [s]
    WheeledRobot.__init__(self, self.wheelDiameterMillimeters/2.0, self.axleLengthMillimeters/2.0)

    # LCM Initialization and Subscription
    self.lc = lcm.LCM()
    lcmMotorSub = self.lc.subscribe("MAEBOT_MOTOR_FEEDBACK", self.motorFeedbackHandler)
    lcmSensorSub = self.lc.subscribe("MAEBOT_SENSOR_DATA", self.sensorDataHandler)
    
    self.lcm_msg_counter = [0, 0]
 
  def calcVelocities(self):
    # IMPLEMENT ME
    # TASK: CALCULATE VELOCITIES FOR ODOMETRY
    # Update self.currOdoVel and self.prevOdoVel
    rclf.currOdoVel = (dxy, dtheta, dt)

  def getVelocities(self):
    # IMPLEMENT ME
    # TASK: RETURNS VELOCITY TUPLE
    # Return a tuple of (dxy [mm], dtheta [rad], dt [s])
    return (dxy, dtheta, dt) # [mm], [rad], [s]

  def calcOdoPosition(self):
    # IMPLEMENT ME
    # TASK: CALCULATE POSITIONS
    # Update self.currOdoPos and self.prevOdoPos
    self.prevOdoPos = self.currOdoPos

  def getOdoPosition(self):
    # IMPLEMENT ME
    # TASK: RETURNS POSITION TUPLE
    # Return a tuple of (x [mm], y [mm], theta [rad])
    return (x, y, theta) # [mm], [rad], [s]

#  def publishOdometry(self):
    # IMPLEMENT ME
    # TASK: PUBLISHES BOT_ODO_POSE MESSAGE
  
#  def publishVelocities(self):
    # IMPLEMENT ME
    # TASK: PUBLISHES BOT_ODO_VEL MESSAGE

  def motorFeedbackHandler(self,channel,data):
    msg = maebot_motor_feedback_t.decode(data)
    # IMPLEMENT ME
    # TASK: PROCESS ENCODER DATA
    # get encoder positions and store them in robot,
    # update robots position and velocity estimate
    self.lcm_msg_counter[0] = self.lcm_msg_counter[0] + 1

  def sensorDataHandler(self,channel,data):
    msg = maebot_sensor_data_t.decode(data)
    # IMPLEMENT ME
    # TASK: PROCESS GYRO DATA
    self.lcm_msg_counter[1] = self.lcm_msg_counter[1] + 1

  def MainLoop(self):
    oldTime = time.time()
    frequency = 20;
    while(1):
      self.lc.handle()
      if(self.lcm_msg_counter[0] * self.lcm_msg_counter[1] > 0):
        #self.publishOdometry()
        #self.publishVelocities()
        oldTime = time.time()
        self.lcm_msg_counter[0] = self.lcm_msg_counter[0] - 1
        self.lcm_msg_counter[1] = self.lcm_msg_counter[1] - 1


if __name__ == "__main__":
  
  robot = Maebot()
  robot.MainLoop()  

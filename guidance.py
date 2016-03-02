import math
import lcm
from lcmtypes import velocity_cmd_t

ANG_TOLERANCE = 1.0 # Degrees
DIST_TOLERANCE = 10.0 # MM
#DIST_TOLERANCE = 5.0 # MM
STOP_TOLERANCE = 1e-6 # TOLERANCE TO CHECK POSITION OR ANGLE DIDNT MOVE
STOP_TIMER = 40 # ITERATIONS UP TO CHECK IF POSITION OR ANGLE IS NOT MOVING

class Guidance():

    # INITIALIZE
    # YOU NEED TO CREATE A GUIDANCE OBJECT IN YOU GROUND STATION
    # CURRENT PLAN IS HARD CODED. NEED TO BE RECEIVED FROM YOUR PLANNING
    # JUST A LIST OF LISTS WITH COORDINATES IN MM    
    def __init__(self):
        self.leg = 0
        self.nleg = 10
        self.state = -1
        self.plan = []
        self.hdg = 0.0
        self.dist = 0.0
        self.command = velocity_cmd_t()
        self.lc = lcm.LCM()
        self.old_dist = 0.0
        self.old_hdg = 0.0
        self.counter = 0

    # I USE THIS FUNCTION SO I CAN HAVE THE GUIDE AND COMMAND
    # ALWAYS RUNNING IN MY GROUND STATION. WHILE IT IS NOT STARTED IT
    # DOES NOTHING
    def start(self):
        self.state = 0
        self.leg = 0
        self.nleg = len(self.plan)

    # CORE OF GUIDANCE
    # SIMPLE STATE MACHINE
    # 0 - CALCULATES NEW HEADING AND COMAMNDS IT
    # 1 - WAIT HEADING TO GET TO THE COMMANDED VALUE OR 
    #     DETECTS ROBOT IS NOT MOVING. BOTH CHANGES TO STATE 2
    # 2 - CALCULATES AND COMMAND NEW FORWARD MOTION
    # 3 - WAIT UNTIL NEW POSITION IS ACHIEVED. UPDATES LEG AND
    #     RESET STATE TO 0
    # NEEDS AN A 3D ARRAY WITH X POS, Y POS AND HDG
    # PUBLISHES COMMAND TO PID
    def guide_and_command(self,pos):
        if (self.state == 0):
            self.hdg = math.atan2((self.plan[self.leg][1] - pos[1]),
                              -(self.plan[self.leg][0] - pos[0]))*180.0/math.pi
            print "Heading:",self.hdg
            print math.fabs(pos[2]*180.0/math.pi)
            if(math.fabs(pos[2]*180.0/math.pi - self.hdg) < ANG_TOLERANCE):
                self.state = 2
            else:
                self.state = 1 
                self.counter = -1 
                self.command.FwdSpeed = 0.0
                self.command.Distance = 0.0
                #Negated some stuff put it back later
                self.command.Angle = self.hdg-pos[2]*180.0/math.pi
                if(self.command.Angle > 0):
                    self.command.AngSpeed = 35.0
                else:
                    self.command.AngSpeed = -35.0
                self.lc.publish("GS_VELOCITY_CMD",self.command.encode())
        elif (self.state == 1):
            if (self.counter % STOP_TIMER == 0):
                self.old_hdg = pos[2]
                self.counter = 0
            if((math.fabs(pos[2]*180.0/math.pi - self.command.Angle) 
                < ANG_TOLERANCE) or
                ((self.counter == STOP_TIMER-1) and 
                (math.fabs(self.old_hdg - pos[2])*180.0/math.pi < STOP_TOLERANCE))):
                self.state = 2
                self.command.AngSpeed = 0.0
                self.command.Angle = 0.0 
                self.lc.publish("GS_VELOCITY_CMD",self.command.encode())
        elif (self.state == 2):
            self.dist = math.sqrt((self.plan[self.leg][1]-pos[1])*
                         (self.plan[self.leg][1]-pos[1]) + 
                         (self.plan[self.leg][0]-pos[0])*
                         (self.plan[self.leg][0]-pos[0]))
            print "Distance:", self.dist
            if (self.dist < DIST_TOLERANCE):
                self.leg += 1
                if(self.leg < self.nleg):
                    self.state = 0
                else:
                    self.state = -1
            else:
                self.state = 3
                self.counter = -1
                self.command.FwdSpeed = 100.0
                self.command.Distance = self.dist
                self.lc.publish("GS_VELOCITY_CMD",self.command.encode())
        elif (self.state == 3):
            if (self.counter % STOP_TIMER == 0):
                self.old_dist = self.dist
                self.counter = 0
            self.dist = math.sqrt((self.plan[self.leg][1]-pos[1])*
                         (self.plan[self.leg][1]-pos[1]) + 
                         (self.plan[self.leg][0]-pos[0])*
                         (self.plan[self.leg][0]-pos[0]))
            if ((self.dist < DIST_TOLERANCE) or
                ((self.counter == STOP_TIMER-1) and 
                (math.fabs(self.dist - self.old_dist) < STOP_TOLERANCE))):
                self.command.FwdSpeed = 0.0
                self.command.Distance = 0.0
                self.lc.publish("GS_VELOCITY_CMD",self.command.encode())         
                self.leg += 1
                if(self.leg < self.nleg):
                    self.state = 0
                else:
                    self.state = -1
        self.counter += 1
        #print self.dist, self.hdg, self.state, self.leg
        #print self.old_dist, self.old_hdg, pos[2]
        #print self.command.Distance, self.command.Angle, self.command.AngSpeed

import sys, os, time
import lcm                                                              
import math
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
 
import matplotlib.backends.backend_agg as agg

import pygame
from pygame.locals import *

from lcmtypes import maebot_diff_drive_t
from lcmtypes import velocity_cmd_t
from lcmtypes import pid_init_t
from lcmtypes import odo_pose_xyt_t
from lcmtypes import odo_dxdtheta_t            
from lcmtypes import rplidar_laser_t

# Callling recently included files
from laser import *
from slam import *
from maps import *
from better_guidance import *
from astar import *

# SLAM preferences
# CHANGE TO OPTIMIZE
USE_ODOMETRY = True           
MAP_QUALITY = 3

# Laser constants
# CHANGE TO OPTIMIZE IF NECSSARY
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance

# Map constants
# CHANGE TO OPTIMIZE
MAP_SIZE_M = 7.0 # size of region to be mapped [m]
INSET_SIZE_M = 1.0 # size of relative map
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
MAP_DEPTH = 3 # depth of data points on map (levels of certainty)

# CONSTANTS
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi

# KWARGS
# PASS TO MAP AND SLAM FUNCTIONS AS PARAMETER
KWARGS, gvars = {}, globals()
for var in ['MAP_SIZE_M','INSET_SIZE_M','MAP_RES_PIX_PER_M','MAP_DEPTH','USE_ODOMETRY','MAP_QUALITY']:
  KWARGS[var] = gvars[var] # constants required in modules

def world_to_astar(x_w, y_w):
  #(y,x)                          
  #x = (MAP_SIZE_M*1000 - y_w)/10
  #y = x_w / 10
  y = int((MAP_SIZE_M*1000-int(x_w)) / 10) #(MAP_SIZE_M*1000 - x_w) / 10         
  x = int(y_w) / 10
  return (y,x)

def astar_to_world(x_a, y_a):
  #(x,y)
  #x = y_a * 10
  #y = (MAP_SIZE_M*1000) - (x_a * 10)
  y = x_a * 10.0#(MAP_SIZE_M*1000) - (x_a * 10)
  x = (MAP_SIZE_M*1000)-(y_a * 10.0)            
  return (x, y)                                       

class MainClass:
    def __init__(self, width=640, height=480, FPS=10):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        
        # LCM Subscribe
        self.lc = lcm.LCM()
        # NEEDS TO SUBSCRIBE TO OTHER LCM CHANNELS LATER!!!
        self.lc.subscribe("ODOMETRY", self.odometryHandler)
        self.odometry = (0,0,0) #x,y,theta

        self.lc.subscribe("VELOCITY", self.velocityHandler)

        self.lc.subscribe("RPLIDAR_LASER", self.LidarHandler)

        # Prepare Figure for Lidar
        self.fig = plt.figure(figsize=[3, 3], # Inches
                              dpi=100)    # 100 dots per inch, 
        self.fig.patch.set_facecolor('white')
        self.fig.add_axes([0,0,1,1],projection='polar')
        self.ax = self.fig.gca()

        # Prepare Figures for control
        path = os.path.realpath(__file__)
        path = path[:-17] + "maebotGUI/"

        self.arrowup = pygame.image.load(path + 'fwd_button.png')
        self.arrowdown = pygame.image.load(path + 'rev_button.png')
        self.arrowleft = pygame.image.load(path + 'left_button.png')
        self.arrowright = pygame.image.load(path + 'right_button.png')
        self.resetbut = pygame.image.load(path + 'reset_button.png')
        self.arrows = [0,0,0,0]

        # PID Initialization - Change for your Gains!
        command = pid_init_t()
        command.kp = 0.0  
        command.ki = 0.0
        command.kd = 0.0
        command.iSat = 0.0 # Saturation of Integral Term. 
                           # If Zero shoudl reset the Integral Term
        self.lc.publish("GS_PID_INIT",command.encode())

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0.0000001

        # Declare Laser, datamatrix and slam
        self.laser = RPLidar(DIST_MIN, DIST_MAX) # lidar
        self.datamatrix = DataMatrix(**KWARGS) # handle map data
        self.slam = Slam(self.laser, **KWARGS) # do slam processing

        self.thetas = []
        self.ranges = []
        self.map_init = False
        self.astar_init = False

        self.guidance = Guidance()
        self.failed_path = True
        self.last_failed_path = time.time()
        self.taken_path = [(350, 350)]
        self.odom_path = []
        self.count = 0
    
    def odometryHandler(self,channel,data):
        msg = odo_pose_xyt_t.decode(data)
        #self.odometry = msg.xyt

    def velocityHandler(self,channel,data):
        msg = odo_dxdtheta_t.decode(data)    
        self.dxy = msg.dxy
        self.dtheta = msg.dtheta
        self.dt = msg.dt
 
    def LidarHandler(self,channel,data): 
        msg = rplidar_laser_t.decode(data)
        self.thetas = msg.thetas                      
        self.ranges = msg.ranges

        #[(mm, deg)], (mm, deg, S)
        self.odometry = self.slam.updateSlam(zip(map(lambda x: x*1000.0, self.ranges), map(math.degrees, self.thetas)), (self.dxy,math.degrees(self.dtheta),self.dt/1000000.0))
        #self.odometry = (self.odometry[0], self.odometry[1], self.odometry[2] + 180.0)
        self.datamatrix.drawBreezyMap(self.slam.getBreezyMap())
        self.datamatrix.getRobotPos(self.odometry,  not self.map_init)
        self.map_init = True    
        self.datamatrix.saveImageFilename("current_map.png")
        
        self.guidance.guide_and_command((self.odometry[1], self.odometry[0], math.radians(self.odometry[2])))


    def MainLoop(self):
        pygame.key.set_repeat(1, 20)
        vScale = 0.5

        # Prepare Text to Be output on Screen
        font = pygame.font.SysFont("DejaVuSans Mono",14)

        mapCounter = 0

        while 1:
            leftVel = 0                               
            rightVel = 0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == KEYDOWN:
                    if ((event.key == K_ESCAPE)
                    or (event.key == K_q)):
                        sys.exit()
                    key = pygame.key.get_pressed()
                    if key[K_RIGHT]:
                        leftVel = leftVel + 0.40
                        rightVel = rightVel - 0.40
                    elif key[K_LEFT]:
                        leftVel = leftVel - 0.40
                        rightVel = rightVel + 0.40
                    elif key[K_UP]:
                        leftVel = leftVel + 0.65
                        rightVel = rightVel + 0.65
                    elif key[K_DOWN]:
                        leftVel = leftVel - 0.65
                        rightVel = rightVel - 0.65
                    else:
                        leftVel = 0.0
                        rightVel = 0.0                      
                    cmd = maebot_diff_drive_t()
                    cmd.motor_right_speed = vScale * rightVel
                    cmd.motor_left_speed = vScale * leftVel
                    self.lc.publish("MAEBOT_DIFF_DRIVE",cmd.encode())
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    command = velocity_cmd_t()
                    command.Distance = 987654321.0
                    if event.button == 1:
                        if ((event.pos[0] > 438) and (event.pos[0] < 510) and
                            (event.pos[1] > 325) and (event.pos[1] < 397)):
                            command.FwdSpeed = 80.0
                            command.Distance = 1000.0
                            command.AngSpeed = 0.0
                            command.Angle = 0.0
                            print "Commanded PID Forward One Meter!"
                        elif ((event.pos[0] > 438) and (event.pos[0] < 510) and
                            (event.pos[1] > 400) and (event.pos[1] < 472)):
                            command.FwdSpeed = -100.0
                            command.Distance = -1000.0
                            command.AngSpeed = 0.0
                            command.Angle = 0.0       
                            print "Commanded PID Backward One Meter!"
                        elif ((event.pos[0] > 363) and (event.pos[0] < 435) and
                            (event.pos[1] > 400) and (event.pos[1] < 472)):
                            command.FwdSpeed = 0.0
                            command.Distance = 0.0
                            command.AngSpeed = 25.0
                            command.Angle = 90.0
                            print "Commanded PID Left One Meter!"
                        elif ((event.pos[0] > 513) and (event.pos[0] < 585) and
                            (event.pos[1] > 400) and (event.pos[1] < 472)):
                            command.FwdSpeed = 0.0
                            command.Distance = 0.0
                            command.AngSpeed = -25.0
                            command.Angle = -90.0
                            print "Commandsed PID Right One Meter!"
                        elif ((event.pos[0] > 513) and (event.pos[0] < 585) and
                            (event.pos[1] > 325) and (event.pos[1] < 397)):
                            pid_cmd = pid_init_t()
                            #pid_cmd.kp = 0.00225       # CHANGE FOR YOUR GAINS!
                            pid_cmd.kp = 0.003    
                            pid_cmd.ki = 0.00001   # See initialization
                            # pid_cmd.kd = 0.000045
                            pid_cmd.kd = 0.00045
                            pid_cmd.iSat = 0.0
                            self.lc.publish("GS_PID_INIT",pid_cmd.encode())
                            print "Commanded PID Reset!"
                        if (command.Distance != 987654321.0):
                            self.lc.publish("GS_VELOCITY_CMD",command.encode())

            # #Handle doing guidance
            # if self.guidance.state == -1:
              # if not self.failed_path or (time.time() - self.last_failed_path) > 5:
                # #Generate a new path
                # startpos = world_to_astar(self.odometry[1], self.odometry[0])
                # endpos = (250, 350)
                # path = astar(self.datamatrix.getMapMatrix(), startpos, endpos)
                # if len(path) > 0:
                  # print "Found a path"                               
                  # self.failed_path = False
                  # pruned_path = [path[0], path[min(5, len(path)-1)]]
                  # self.guidance.plan = map(lambda x: astar_to_world(x[1], x[0]), pruned_path)
                  # #print path
                  # print self.guidance.plan
                  # self.guidance.start()
                # else:
                  # print "Failed to find a path"
                  # self.failed_path = True
                  # self.last_failed_path = time.time()
            # else:
              # self.guidance.guide_and_command((self.odometry[1], self.odometry[0], math.radians(self.odometry[2])))
            
             #Handle doing guidance - Kevin       
            if self.guidance.state == -1:                                        
              if not self.failed_path or (time.time() - self.last_failed_path) > 1:
                #Generate a new path 
                startpos = world_to_astar(self.odometry[1], self.odometry[0]) #(x,y) -> y*,x*
                #endpos=(250.250)             
                #robot will start at (3500,3500) (x,y)
                #endpos = world_to_astar(3030,5500) #x, y - >y*,x* 
                endpos = world_to_astar(2500,3500)
                print "Starting at: ", startpos, "Ending at: ", endpos
                #print "Checking for a path. Printing..." 
                self.odom_path.append(world_to_astar(self.odometry[1], self.odometry[0]))
                #print "odom:", self.odom_path
                path = astar(self.datamatrix.getMapMatrix(), startpos, endpos, False)
                if len(path) > 0:
                  print "Found a path. This is the guidance:"    
                  self.failed_path = False 
                  #pruned_path = [path[min(2, len(path)-1)], path[min(5, len(path)-1)]] 
                  #pruned_path = [path[min(5, len(path)-2)], path[min(10, len(path)-2)]] #
                  #self.guidance.plan = map(lambda x: astar_to_world(x[1], x[0]), pruned_path) #y*,x* -> x, y                                       
                  #print self.guidance.plan
                  #print self.guidance.plan     
                  next_point = path[min(len(path)-1, 5)]
                  print next_point
                  self.guidance.next_point = astar_to_world(next_point[1], next_point[0])
                  print self.guidance.next_point
                  #print self.guidance.next_point
                  #self.taken_path.append(next_point)
                  self.guidance.start()     
                  #print "Planned:", self.taken_path
                  #numpy.save("failed_path.npy", self.datamatrix.getMapMatrix())
                else:                       
                  print "Failed to find a path"
                  self.failed_path = True             
                  self.last_failed_path = time.time()      
                #numpy.save("temp_path_%d.npy"%(self.count), self.datamatrix.getMapMatrix())
                #print self.odometry
                
                pid_cmd = pid_init_t()
                #pid_cmd.kp = 0.00225       # CHANGE FOR YOUR GAINS!
                pid_cmd.kp = 0.003    
                pid_cmd.ki = 0.00001   # See initialization
                # pid_cmd.kd = 0.000045
                pid_cmd.kd = 0.00045
                pid_cmd.iSat = 0.0
                self.lc.publish("GS_PID_INIT",pid_cmd.encode())                                         
                                      
#            else:                           
##              print "ODO"
##              print (self.odometry[1], self.odometry[0])                       
##              print "Guidance"
##              print self.guidance.plan                       
##              self.guidance.guide_and_command((self.odometry[1], self.odometry[0], math.radians(self.odometry[2])))
#              self.path_failed = True
#              self.last_failed_path = time.time()
                                                        
            self.screen.fill((255,255,255))

            # Plot Lidar Scans 
            plt.cla()          
            self.ax.plot(self.thetas,self.ranges,'or',markersize=2)

            self.ax.set_rmax(1.5)            
            self.ax.set_theta_direction(-1)
            self.ax.set_theta_zero_location("N")
            self.ax.set_thetagrids([0,45,90,135,180,225,270,315],
                                    labels=['','','','','','','',''], 
                                    frac=None,fmt=None)                 
            self.ax.set_rgrids([0.5,1.0,1.5],labels=['0.5','1.0',''],
                                    angle=None,fmt=None)

            canvas = agg.FigureCanvasAgg(self.fig)
            canvas.draw()        
            renderer = canvas.get_renderer()
            raw_data = renderer.tostring_rgb()
            size = canvas.get_width_height()
            surf = pygame.image.fromstring(raw_data, size, "RGB")
            self.screen.blit(surf, (320,0))
                                                  
            #Put map on screen 
            if self.map_init == True:                 
              image = pygame.image.load("current_map.png")
              image = pygame.transform.scale(image, (320, 320))
              self.screen.blit(image, (0,0))      

                                                                           
            # Position and Velocity Feedback Text on Screen
            self.lc.handle()          
            pygame.draw.rect(self.screen,(0,0,0),(5,350,300,120),2)
            text = font.render("    POSITION    ",True,(0,0,0))
            self.screen.blit(text,(10,360))
            text = font.render("x: %.2f [mm]" % (self.odometry[1]),True,(0,0,0))
            self.screen.blit(text,(10,390))
            text = font.render("y: %.2f [mm]" % (self.odometry[0]),True,(0,0,0))
            self.screen.blit(text,(10,420))
            text = font.render("t: %.2f [rad]" % (self.odometry[2]),True,(0,0,0))
            self.screen.blit(text,(10,450))
                               
            text = font.render("    VELOCITY    ",True,(0,0,0))
            self.screen.blit(text,(150,360))
            text = font.render("dxy/dt: %.2f [mm/us]" % (self.dxy / self.dt * 1000000),True,(0,0,0))
            self.screen.blit(text,(150,390))
            text = font.render("dth/dt: %.2f [deg/us]" % (self.dtheta / self.dt * 1000000),True,(0,0,0))
            self.screen.blit(text,(150,420))
            text = font.render("dt: %d [s]" % (self.dt/1000000),True,(0,0,0))
            self.screen.blit(text,(150,450))

            # Plot Buttons
            self.screen.blit(self.arrowup,(438,325))
            self.screen.blit(self.arrowdown,(438,400))
            self.screen.blit(self.arrowleft,(363,400))
            self.screen.blit(self.arrowright,(513,400))
            self.screen.blit(self.resetbut,(513,325))
                                        
            pygame.display.flip()

            #if self.map_init:
            #  self.datamatrix.saveImage()



MainWindow = MainClass()
MainWindow.MainLoop()

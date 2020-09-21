####################
# main.py
# this file defines the main loop of the simulator,
# as well as the 3D rendering and visualization
#
# Author: Jianyu Chen
# Copyright: 2016
####################

#from pandac.PandaModules import loadPrcFileData
#loadPrcFileData('', 'load-display tinydisplay')
from __future__ import division
import sys
import os
import numpy as np
import time
from scipy import io

import direct.directbase.DirectStart
from direct.showbase.DirectObject import DirectObject
from direct.showbase.InputStateGlobal import inputState
from direct.interval.IntervalGlobal import *
from direct.gui.DirectGui import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.actor import Actor
from random import *

from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32
from panda3d.core import *
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletVehicle
from panda3d.bullet import ZUp

from vehicle import *
from road import *
from sensor import *
from agent import *
import matplotlib.pyplot as plt


class Game(DirectObject):

  def __init__(self):
    base.setBackgroundColor(0.1, 0.1, 0.8, 1)
    base.setFrameRateMeter(True)

    # road geometry generation
    #road=trail(100,50,40,0.2) # generating a circular lane track
    self.precision=0.2 # length of a piece of centerLine
    self.texLength=10
    self.laneNum=4
    self.radiu=500
    road=basicFreeWay(100,self.radiu,1,self.precision,2) # generating a piece of freeway
    #road=straightCenter(np.array([0,0]),math.pi/2,2000,2)  # generating a straight way
    self.segLine=road.getLine() # the centerLine
    self.road=roadGenerator(np.array([0,-1]),8,2,road.getFollowing(),self.segLine,-1,self.texLength,self.precision) # generate road polygon
    self.rightBound=self.road.rightPoints
    self.leftBound=self.road.leftPoints
    segLength=len(self.segLine)
    self.lines=[]
    for j in range(0,self.laneNum):
        line=[]
        for i in range(0,segLength):
            line.append(np.array([self.rightBound[i][0]*(1/8+j*1/4)*1+self.leftBound[i][0]*(7/8-j*1/4)*1,self.rightBound[i][1]*(1/8+j*1/4)+self.leftBound[i][1]*(7/8-j*1/4)]))
        self.lines=self.lines+[line]
            
      #self.rightLine.append(self.segLine[i]/2+self.rightBound[i]/2)
      #self.leftLine.append(self.segLine[i]/2+self.leftBound[i]/2)
      
    node=self.road.getNode()

    # road texture
    floorTex = loader.loadTexture('maps/street4.jpg')
    floor = render.attachNewNode(node)
    floor.setTexture(floorTex)
    #floor.flattenStrong()
    
    
    # initial automated vehicle   
    self.initAV=[]
    self.agents=[]
    
    
    self.scenario = 0
         
    # Overtaking
    if self.scenario is 0:      
      desiredV=10
      self.horizon = 30
      self.ts = 0.2
      # car 0 **
      self.initAV.append([0,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
      # car 1 **
      self.initAV.append([15,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
      # car 2 **
      self.initAV.append([20,-2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=1))
      # car 3 **
      self.initAV.append([25,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))   
      

    # Crossing
    elif self.scenario is 1:
      desiredV=10   
      self.horizon = 20
      self.ts = 0.1
      # car 0 **
      self.initAV.append([0,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))
      # car 1 **
      self.initAV.append([0,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))    


    # Platoon formation
    elif self.scenario is 2:
      desiredV=20      
      self.horizon = 20
      self.ts = 0.1
      # car 0 **
      self.initAV.append([0,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))
      # car 1 **
      self.initAV.append([6,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
      # car 2 **
      self.initAV.append([12,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0)) 
      # car 3 **
      self.initAV.append([18,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))  
      
      
    # Deadlock breaking
    # Merging
    elif self.scenario is 3:      
      desiredV=10   
      self.horizon = 25
      self.ts = 0.1   
      # car 0 **
      self.initAV.append([0,-2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=1)) 
      # car 1 **
      self.initAV.append([6,-2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=1))
      # car 2 **
      self.initAV.append([0,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))          
      # car 3 **
      self.initAV.append([6,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
      
      
    elif self.scenario is 4:      
      desiredV=10   
      self.horizon = 20
      self.ts = 0.1   
      # car 0 **
      self.initAV.append([0,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))          
      # car 1 **
      self.initAV.append([0,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0)) 
      
      
    elif self.scenario is 5:      
      desiredV=10   
      self.horizon = 25
      self.ts = 0.1   
      # car 0 **
      self.initAV.append([0,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))          
      # car 1 **
      self.initAV.append([0,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
      # car 2 **
      self.initAV.append([6,-6,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0)) 
      # car 3 **
      self.initAV.append([6,2,desiredV])
      self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2)) 
      
      
    # DMPC vs MCCFS
    elif self.scenario is 6:
      desiredV=20      
      self.horizon = 20
      self.ts = 0.1
      num_veh = 10
      for i in range(num_veh):
          self.initAV.append([6*i,-2,desiredV])
          self.agents.append(dcfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))          

#    # C-MPC  
#    elif self.planning_mode is 1:
#        # Overtaking
#        if self.scenario is 0:      
#          desiredV=20
#          self.horizon = 40
#          self.ts = 0.05
#          # car 0 **
#          self.initAV.append([0,-2,desiredV])
#          self.agents.append(ccfsAgent(vGain=50,thetaGain=1000,desiredV=desiredV,laneId=1))
#          # car 1 **
#          self.initAV.append([10,-2,desiredV])
#          self.agents.append(ccfsAgent(vGain=50,thetaGain=1000,desiredV=desiredV,laneId=1))
##          # car 2 **
##          self.initAV.append([30,2,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
##          # car 3 **
##          self.initAV.append([40,-6,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))    
##          # car 4 **
##          self.initAV.append([60,-2,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=1))
#
#
#        # Crossing: collision due to no adding priority constraints. Ref: DSCC20
#        elif self.scenario is 1:
#          desiredV=10   
#          self.horizon = 30
#          self.ts = 0.02
#          # car 0 **
#          self.initAV.append([0,-6,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))
#          # car 1 **
#          self.initAV.append([0,2,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))    
#
#    
#        # Platoon formation
#        elif self.scenario is 2:
#          desiredV=20      
#          self.horizon = 20
#          self.ts = 0.05
#          # car 0 **
#          self.initAV.append([0,-6,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))
#          # car 1 **
#          self.initAV.append([6,2,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
#          # car 2 **
#          self.initAV.append([12,-6,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0)) 
#          # car 3 **
#          self.initAV.append([18,2,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))    
#    
#    
#        # Platoon formation: no deadlock
#        elif self.scenario is 3:      
#          desiredV=20   
#          self.horizon = 20
#          self.ts = 0.05
#          # car 0 **
#          self.initAV.append([0,-6,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))
#          # car 1 **
#          self.initAV.append([0,2,desiredV])
#          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
#          # car 2 **
##          self.initAV.append([0,6,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=3))
#
#
##        # Merging
##        elif self.scenario is 4:      
##          desiredV=10   
##          self.horizon = 25
##          self.ts = 0.1   
##          # car 0 **
##          self.initAV.append([0,-6,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0))          
##          # car 1 **
##          self.initAV.append([0,2,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2))
##          # car 2 **
##          self.initAV.append([6,-6,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=0)) 
##          # car 3 **
##          self.initAV.append([6,2,desiredV])
##          self.agents.append(ccfsAgent(vGain=5,thetaGain=50,desiredV=desiredV,laneId=2)) 
#                                 
#    self.pos = np.zeros((self.num_veh, self.dim))
#    self.ref_path = np.zeros((self.num_veh, self.horizon, self.dim))
          
    self.replanFlag = False
    self.traj_log = []
    self.traj_plan = []
    self.planning_time_log = []
    self.dim = 2
    self.num_veh = len(self.initAV)    
    self.shared_path = np.zeros((self.num_veh, self.horizon, self.dim))
    self.k = 0

    
    # initial camera
    base.cam.setPos(0, -20, 4)
    base.cam.lookAt(0, 0, 0)

    # Light
    alight = AmbientLight('ambientLight')
    alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
    alightNP = render.attachNewNode(alight)

    dlight = DirectionalLight('directionalLight')
    dlight.setDirection(Vec3(1, 1, -1))
    dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
    dlightNP = render.attachNewNode(dlight)

    render.clearLight()
    render.setLight(alightNP)
    render.setLight(dlightNP)

    # Input setup
    self.accept('escape', self.doExit)
    self.accept('r', self.doReset)
    self.accept('f1', self.toggleWireframe)
    self.accept('f2', self.toggleTexture)
    self.accept('f3', self.toggleDebug)
    self.accept('f5', self.doScreenshot)
    self.accept('u', self.doLaneChangeLeft)
    self.accept('o', self.doLaneChangeRight)
    self.accept('c', self.doChangeDesigned)
    
    # Task manager
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____
  def doChangeDesigned(self):
      print('Do lang changing')
      if self.scenario is 0:
          self.agents[0].desiredV = 50
          
      elif self.scenario is 1:
          self.vehicles[0].agent.targetLane=2
          self.vehicles[1].agent.targetLane=0
          
      else:
          for i in range(len(self.initAV)):
              self.vehicles[i].agent.targetLane=1


  def doLaneChangeLeft(self):
      if self.vehicles[0].agent.targetLane>=1:
          self.vehicles[0].agent.targetLane=self.vehicles[0].agent.targetLane-1

  def doLaneChangeRight(self):
      if self.vehicles[0].agent.targetLane<=2:
          self.vehicles[0].agent.targetLane=self.vehicles[0].agent.targetLane+1

  def doExit(self):
    self.cleanup()
    sys.exit(1)

  def doReset(self):
    self.cleanup()
    self.setup()
    self.agents[0].previousInput=[0,0]

  def toggleWireframe(self):
    base.toggleWireframe()

  def toggleTexture(self):
    base.toggleTexture()

  def toggleDebug(self):
    if self.debugNP.isHidden():
      self.debugNP.show()
    else:
      self.debugNP.hide()

  def doScreenshot(self):
    base.screenshot('Bullet')

  # exit
  def cleanup(self):
    self.world = None
    self.worldNP.removeNode()
    if len(self.traj_log)>0:
        length = len(self.traj_log)
        num_time_step = int(length/(self.dim*self.num_veh))
        self.traj_log = np.reshape(self.traj_log,(num_time_step,self.dim*self.num_veh))
        self.traj_plan = np.reshape(self.traj_plan,(num_time_step*self.num_veh,self.dim*self.horizon))
#        io.savemat('traj.mat', {'traj_log': self.traj_log})
#        io.savemat('traj_plan.mat', {'traj_plan': self.traj_plan})
#        io.savemat('planning_time.mat', {'planning_time': self.planning_time_log[1:]})
        
    
  # control camera
  def updateCamera(self):
    followVehicle = 0
    direction = self.agents[followVehicle].getPreview(0, 0)[0]
    direction = direction / np.linalg.norm(direction)
    position=self.vehicles[followVehicle].getPosVector()
    #camera
    base.cam.setPos(position[0], position[1]-15*direction[1], 4)
    base.cam.lookAt(position)
    

    



#  # MCCFS from DSCC20
#  # update path with centralized multi car planner:
#  # get reference multi path
#  # call CFS_planner.Plan_trajectory
#  # update path for every agent
#  def updatePath(self):
#    num_cars = len(self.initAV)
#    if self.replanFlag is True:
#      multi_path = np.zeros((num_cars, self.num_steps, 2))
#      multi_path_log = np.zeros((num_cars, self.num_steps, 2))
#      for i in range(num_cars):
#        if self.agents[i].traj is not None:
#          n = self.agents[i].traj.shape[0]
#          multi_path[i][:n] = self.agents[i].traj[:n]
#          multi_path[i][n:] = self.agents[i].getPreview2([self.agents[i].targetLane],[self.num_steps])[n:]
#        else:
#          multi_path[i] = self.agents[i].getPreview2([self.agents[i].targetLane],[self.num_steps])
#
#      if self.changeFlag is True:
#        if self.scenario is 0:
#          # 2 cars take over with different desired velocity scenario -----
#          self.agents[0].desiredV = 25
#          multi_path[0][2:] = self.agents[0].getPreview2([1,0,1], [10, 20, self.num_steps-10-20])[2:]
#          # ----- 2 cars take over
#        elif self.scenario is 1:
#          # 9 cars scenario -----
#          multi_path[0][2:] = self.agents[0].getPreview2([0,1], [10,self.num_steps-10])[2:]
#          self.agents[0].setTargetLane(1)
#          multi_path[1][2:] = self.agents[1].getPreview2([0,1], [5,self.num_steps-5])[2:]
#          self.agents[1].setTargetLane(1)
#          multi_path[3][2:] = self.agents[3].getPreview2([1,2], [5,self.num_steps-5])[2:]
#          self.agents[3].setTargetLane(2)
#          # ----- 9 cars scenario
#        self.changeFlag = False
#
#      new_path = CFS_planner.Plan_trajectory(self.MAX_ITER, multi_path, self.min_dist)
#
#      for i in range(num_cars):
#        car_path = np.zeros((self.num_steps, 2))
#        car_path[:, 0] = new_path[2*i : : num_cars*2]
#        car_path[:, 1] = new_path[2*i+1 : : num_cars*2]
#        self.agents[i].traj = car_path
#        multi_path_log[i] = car_path
#      self.traj_log.append(multi_path_log[:,2:,:])
#      self.replanFlag = False
#
#    else:
#      forwardFlag = False
#      for i in range(num_cars):
#        if self.agents[i].traj is not None and len(self.agents[i].traj)>2:
#          pos = self.agents[i].getState()[0]
#          slope = self.agents[i].traj[1] - self.agents[i].traj[0]
#          constant = -slope.dot(self.agents[i].traj[0])
#          if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
#              forwardFlag = True
#              break
#        else:
#          self.replanFlag = True
#          break
#      if forwardFlag is True:
#        for i in range(num_cars):
#          self.agents[i].traj = self.agents[i].traj[1:]    
#
#  def updateReplayPath(self):
#    if self.replayIndex < self.replayTrajectories.shape[1]-1:
#      forwardFlag = False
#      for i in range(len(self.initAV)):
#        pos = self.agents[i].getState()[0]
#        slope = self.agents[i].traj[1] - self.agents[i].traj[0]
#        constant = -slope.dot(self.agents[i].traj[0])
#        if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
#            forwardFlag = True
#            break
#      if forwardFlag is True:
#        for i in range(len(self.initAV)):
#          self.agents[i].traj = self.replayTrajectories[i,self.replayIndex:self.replayIndex+2,:]
#        self.replayIndex+=1
#        if self.replayIndex == self.changeIdx:
#          self.doLaneChangeDesigned()
#    else:
#      self.doExit()
#
#  def updateLog(self):
#    multi_path_log = self.traj_log.pop()
#    remaining = self.agents[0].traj.shape[0]
#    multi_path_log = multi_path_log[:,:-remaining,:]
#    self.traj_log.append(multi_path_log)
#    idx = 0
#    for i in range(len(self.traj_log)):
#      idx+=self.traj_log[i].shape[1]
#    self.changeIdx.append(idx)   



#  # C-MPC    
#  def Central_planner(self):    
#      if self.replanFlag is True:
#          self.Central_planner_replan()
#          self.replanFlag = False
#      else:          
#          forwardFlag = False
#          for i in range(len(self.initAV)):
#            if self.agents[i].traj is not None and len(self.agents[i].traj)>2:
#              pos = self.agents[i].getPos()
#              slope = self.agents[i].traj[1] - self.agents[i].traj[0]
#              constant = -slope.dot(self.agents[i].traj[0])
#              if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
#                  forwardFlag = True
#                  break
#            else:
#              self.replanFlag = True
#              break
#          if forwardFlag is True:
#            for i in range(len(self.initAV)):
#              self.agents[i].traj = self.agents[i].traj[1:]  
#                            
#      # Control
#      for i in range(len(self.initAV)):
#          self.vehicles[i].controlInput(self.vehicles[i].agent.doControl())               
#  
#
#  
#  def Central_planner_replan(self):
#      start = time.perf_counter()
#      
##      # Receive
##      for i in range(len(self.initAV)):
##          self.shared_path[i] = self.agents[i].Communication_Sender()
#      
#      # Get position
#      for i in range(len(self.initAV)):
#          self.pos[i] = self.agents[i].getPos()
#      # Get ref traj
#      for i in range(len(self.initAV)):
#          self.ref_path[i] = self.agents[i].getRef()
#      print('Ref:',self.ref_path)
#      
#      # Planning
#      self.shared_path = DCFS.Centralized_solver(self.pos, self.ref_path, self.num_veh, self.horizon, self.dim)
#          
#      end = time.perf_counter()
#      print('Planning time:',end-start)
#      
#      # Send traj
#      for i in range(len(self.initAV)):
#          self.agents[i].Communication_Receiver(self.shared_path[i]) 
#      
##      # Control
##      for i in range(len(self.initAV)):
##          self.vehicles[i].controlInput(self.vehicles[i].agent.doControl()) 


    
  def Communication(self):
      # Send
      for i in range(len(self.initAV)):
          self.shared_path[i] = self.agents[i].Communication_Sender()
      # Receive
      for i in range(len(self.initAV)):
          self.agents[i].Communication_Receiver(self.shared_path) 
    
  
  # simulation update per step
  def update(self, task):
#    dt = globalClock.getDt()
#    print(dt) # ~ 0.02s
    
#    if self.planning_mode is 0:         
#        if self.replanFlag is False:
#            print('False')                 
#            for i in range(len(self.initAV)):              
#                if self.agents[i].traj is not None and len(self.agents[i].traj)>2:
#                    pos = self.agents[i].getPos()
#                    slope = self.agents[i].traj[1] - self.agents[i].traj[0]
#                    constant = -slope.dot(self.agents[i].traj[0])
#                    if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
#                        self.replanFlag = True
#                        break                                   
        
    # Share path
    self.Communication()
    
    for i in range(len(self.initAV)):
        # Traj log
        self.traj_log = np.append(self.traj_log,(self.agents[i].getPos()))
        
        self.traj_plan = np.append(self.traj_plan,self.agents[i].Communication_Sender())
        
        # Distributed planning and control
        self.vehicles[i].controlInput(self.vehicles[i].agent.doControl(self.scenario)) 
        # Planning time log
        self.planning_time_log = np.append(self.planning_time_log,(self.agents[i].getPlanningTime()))
    
        
#    time.sleep(0.2);
#    print('------time sleep------')
        
#    self.k = self.k+1
#    if self.k is 60:
#     self.cleanup()
        
    self.world.doPhysics(1.6)
    self.updateCamera()     
    return task.cont



  # physical world setup
  def setup(self):
    self.worldNP = render.attachNewNode('World')

    # World
    self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
    # self.debugNP.show()

    self.world = BulletWorld()
    self.world.setGravity(Vec3(0, 0, -9.81))
    self.world.setDebugNode(self.debugNP.node())

    # Plane
    shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

    np = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
    np.node().addShape(shape)
    np.setPos(0, 0, -1)
    np.setCollideMask(BitMask32.allOn())

    self.world.attachRigidBody(np.node())

    # initial vehicles
    length=2.8
    width=1.2
    height=1
    axisDis=2.1
    wheelDis=1.4
    wheelH=0.3
    radius=0.25
    
    self.vehicles=[]
    self.sensors=[]
    
    # Adding autonomous vehicles
    for i in range(len(self.initAV)):
        self.vehicles.append(basicVehicle(self,[self.initAV[i][0],self.initAV[i][1],-0.6],self.initAV[i][2],length,width,height,axisDis,wheelDis,radius,wheelH))
        self.sensors.append(basicSensor(self))  # initial sensor
        self.sensors[i].setVehicle(self.vehicles[i])
        self.vehicles[i].setSensor(self.sensors[i])
        self.vehicles[i].sensor.align()
        self.agents[i].setVehicle(self.vehicles[i])
        self.vehicles[i].setAgent(self.agents[i])
        self.agents[i].setVehicleIndex(i)        
        self.agents[i].horizon = self.horizon
        self.agents[i].ts = self.ts
        self.agents[i].traj = self.agents[i].getRef()

    print('Set up completed')
    
game = Game()
base.run()

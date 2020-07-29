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
import direct.directbase.DirectStart
import numpy as np
import math

from direct.showbase.DirectObject import DirectObject
from direct.showbase.InputStateGlobal import inputState

from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32

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
import CFS_planner
import matplotlib.pyplot as plt

from panda3d.core import *
import sys
import os

import direct.directbase.DirectStart
from direct.interval.IntervalGlobal import *
from direct.gui.DirectGui import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.actor import Actor
from random import *



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
    road=basicFreeWay(200,self.radiu,1,self.precision,2) # generating a piece of freeway
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
    
    # grass background generation
    '''floorTex1 = loader.loadTexture('maps/envir-ground.jpg')
    cm1 = CardMaker('')
    cm1.setFrame(-2, 2, -2, 2)
    floor1 = render.attachNewNode(PandaNode("floor1"))
    for y in range(400):
        for x in range(22):
            nn1 = floor1.attachNewNode(cm1.generate())
            nn1.setP(-90)
            nn1.setPos((x - 10) * 4, (y - 20) * 4, -1.1)
    floor1.setTexture(floorTex1)
    floor1.flattenStrong()'''

    # central planner settings
    self.scenario = 0                   # specify a scenario
    self.replayFile = "traj_log_2.npz"  # specify a filename to replay

    self.replanFlag = True
    self.changeFlag = False
    self.MAX_ITER = 20
    self.min_dist = 3.9
    self.num_steps = 20
    self.replayTrajectories = None
    self.replayIndex = 0
    self.changeIdx = []

    if self.replayFile is "traj_log_2.npz":
      self.scenario = 0
    elif self.replayFile is "traj_log_9.npz":
      self.scenario = 1
    self.traj_log = []


    
    # initial automated vehicle
    self.scenario = 0
    
    self.initAV=[]
    self.agents=[]

    if self.scenario is 0:
      # 2 cars take over with different desired velocity scenario -----
      desiredV=10
      
      # car 0 **
      self.initAV.append([0,-2,desiredV])
      self.agents.append(dcfsAgent(vGain=2,thetaGain=100,desiredV=desiredV,laneId=1))
      # car 1 **
      self.initAV.append([20,-2,desiredV])
      self.agents.append(dcfsAgent(vGain=2,thetaGain=100,desiredV=desiredV,laneId=1))
      
      # ----- 2 cars take over'''
    elif self.scenario is 1:
      # 9 cars scenario -----
      desiredV=20
      self.num_steps = 20
      # car 0 **
      self.initAV.append([0,-6,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=0))
      # car 1 **
      self.initAV.append([10,-6,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=0))
      # car 2
      self.initAV.append([14,-6,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=0))
      # car 3 **
      self.initAV.append([2,-2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=1))
      # car 4
      self.initAV.append([15,-2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=1))
      # car 5
      self.initAV.append([25,-2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=1))
      # car 6
      self.initAV.append([5,2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=2))
      # car 7
      self.initAV.append([10,2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=2))
      # car 8
      self.initAV.append([20,2,10])
      self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=2))
      # ----- 9 cars scenario'''
    
    self.dim = 2
    self.num_veh = len(self.initAV)
    self.horizon = self.agents[0].horizon
    self.shared_path = np.zeros((self.num_veh, self.horizon, self.dim))

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
    self.accept('c', self.doLaneChangeDesigned)

    # inputState.watchWithModifiers('forward', 'w')
    # inputState.watchWithModifiers('reverse', 's')
    # inputState.watchWithModifiers('turnLeft', 'a')
    # inputState.watchWithModifiers('turnRight', 'd')
    # inputState.watchWithModifiers('brake1', 'x')

    # inputState.watchWithModifiers('For', 'i')
    # inputState.watchWithModifiers('Back', 'k')
    # inputState.watchWithModifiers('Lef', 'j')
    # inputState.watchWithModifiers('Righ', 'l')
    # inputState.watchWithModifiers('brake2', 'space')
    

    # Task manager
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____
  def doLaneChangeDesigned(self):
#    if self.replayFile is None:
#      self.replanFlag = True
#      self.changeFlag = True
#      self.updateLog()
#    else:
      if self.scenario is 0:
          self.agents[0].desiredV = 3*self.agents[0].desiredV
#      print("Do lane change")

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


  # control camera
  def updateCamera(self):
    followVehicle = 0
    direction = self.agents[followVehicle].getPreview(0, 0)[0]
    direction = direction / np.linalg.norm(direction)
    position=self.vehicles[followVehicle].getPosVector()
    #camera
    base.cam.setPos(position[0], position[1]-15*direction[1], 4)
    base.cam.lookAt(position)
    
    # #current state of vehicle
    # direction=self.vehicles[0].getDirection()
    # position=self.vehicles[0].getPosVector()
    # #camera
    # base.cam.setPos(position[0]-15*direction[0], position[1]-15*direction[1], 4)
    # base.cam.lookAt(position)
    

  # update path with centralized multi car planner:
  # get reference multi path
  # call CFS_planner.Plan_trajectory
  # update path for every agent
  def updatePath(self):
    num_cars = len(self.initAV)
    if self.replanFlag is True:
      multi_path = np.zeros((num_cars, self.num_steps, 2))
      multi_path_log = np.zeros((num_cars, self.num_steps, 2))
      for i in range(num_cars):
        if self.agents[i].traj is not None:
          n = self.agents[i].traj.shape[0]
          multi_path[i][:n] = self.agents[i].traj[:n]
          multi_path[i][n:] = self.agents[i].getPreview2([self.agents[i].targetLane],[self.num_steps])[n:]
        else:
          multi_path[i] = self.agents[i].getPreview2([self.agents[i].targetLane],[self.num_steps])

      if self.changeFlag is True:
        if self.scenario is 0:
          # 2 cars take over with different desired velocity scenario -----
          self.agents[0].desiredV = 25
          multi_path[0][2:] = self.agents[0].getPreview2([1,0,1], [10, 20, self.num_steps-10-20])[2:]
          # ----- 2 cars take over'''
        elif self.scenario is 1:
          # 9 cars scenario -----
          multi_path[0][2:] = self.agents[0].getPreview2([0,1], [10,self.num_steps-10])[2:]
          self.agents[0].setTargetLane(1)
          multi_path[1][2:] = self.agents[1].getPreview2([0,1], [5,self.num_steps-5])[2:]
          self.agents[1].setTargetLane(1)
          multi_path[3][2:] = self.agents[3].getPreview2([1,2], [5,self.num_steps-5])[2:]
          self.agents[3].setTargetLane(2)
          # ----- 9 cars scenario'''
        self.changeFlag = False

      new_path = CFS_planner.Plan_trajectory(self.MAX_ITER, multi_path, self.min_dist)

      for i in range(num_cars):
        car_path = np.zeros((self.num_steps, 2))
        car_path[:, 0] = new_path[2*i : : num_cars*2]
        car_path[:, 1] = new_path[2*i+1 : : num_cars*2]
        self.agents[i].traj = car_path
        multi_path_log[i] = car_path
      self.traj_log.append(multi_path_log[:,2:,:])
      self.replanFlag = False

    else:
      forwardFlag = False
      for i in range(num_cars):
        if self.agents[i].traj is not None and len(self.agents[i].traj)>2:
          pos = self.agents[i].getState()[0]
          slope = self.agents[i].traj[1] - self.agents[i].traj[0]
          constant = -slope.dot(self.agents[i].traj[0])
          if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
              forwardFlag = True
              break
        else:
          self.replanFlag = True
          break
      if forwardFlag is True:
        for i in range(num_cars):
          self.agents[i].traj = self.agents[i].traj[1:]    

  def updateReplayPath(self):
    if self.replayIndex < self.replayTrajectories.shape[1]-1:
      forwardFlag = False
      for i in range(len(self.initAV)):
        pos = self.agents[i].getState()[0]
        slope = self.agents[i].traj[1] - self.agents[i].traj[0]
        constant = -slope.dot(self.agents[i].traj[0])
        if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
            forwardFlag = True
            break
      if forwardFlag is True:
        for i in range(len(self.initAV)):
          self.agents[i].traj = self.replayTrajectories[i,self.replayIndex:self.replayIndex+2,:]
        self.replayIndex+=1
        if self.replayIndex == self.changeIdx:
          self.doLaneChangeDesigned()
    else:
      self.doExit()

  def updateLog(self):
    multi_path_log = self.traj_log.pop()
    remaining = self.agents[0].traj.shape[0]
    multi_path_log = multi_path_log[:,:-remaining,:]
    self.traj_log.append(multi_path_log)
    idx = 0
    for i in range(len(self.traj_log)):
      idx+=self.traj_log[i].shape[1]
    self.changeIdx.append(idx)
    
    
    
    
    
    
  def Communication(self):
      # Send
      for i in range(len(self.initAV)):
          self.shared_path[i] = self.agents[i].Communication_Sender()
      # Receive
      for i in range(len(self.initAV)):
          self.agents[i].Communication_Receiver(self.shared_path) 
#      print('path[0]:',self.shared_path[0])
    
  
  # simulation update per step
  def update(self, task):
#    dt = globalClock.getDt()
#    print(dt) # ~ 0.02s

    # Share path
    self.Communication()
    # Distributed planning and control
    for i in range(len(self.initAV)):
        self.vehicles[i].controlInput(self.vehicles[i].agent.doControl())     
    

    self.world.doPhysics(1.6)

    self.updateCamera() 
    
    return task.cont

  # exit
  def cleanup(self):
    self.world = None
    self.worldNP.removeNode()
    if len(self.traj_log)>0:
      log = np.concatenate(self.traj_log, axis=1)
      np.savez("traj_log", c=self.changeIdx, log=log)

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
        self.agents[i].traj = self.agents[i].getRef()
#        print('i:',self.agents[i].veh_index)

        
    print('Set up completed')
    
game = Game()
base.run()

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
    self.precision=1 # length of a piece of centerLine
    self.texLength=10
    self.laneNum=4
    self.radiu=500
    road=basicFreeWay(200,self.radiu,3,self.precision,2) # generating a piece of freeway
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

    # initial automated vehicle
    desiredV=40
    self.initAV=[]
    self.agents=[]
    self.initAV.append([10,-6,30])
    self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=0))
    self.initAV.append([10,-2,30])
    self.agents.append(mccfsAgent(vGain=20,thetaGain=1000,desiredV=desiredV,laneId=1))
    self.replanFlag = True
    # self.agents.append(planningAgent(20,5,desiredV,int(math.floor((self.initAV[1]+8)/4)),surroundingVehicleNum,self.radiu)) # initial agent
    # self.agents.append(previewAgent(20,5,desiredV,int(math.floor((self.initAV[1]+8)/4)))) # ctl test
    # self.agents.append(autoBrakeAgent(20,5,desiredV)) # ctl test

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
    #current state of vehicle
    direction=self.vehicles[0].getDirection()
    position=self.vehicles[0].getPosVector()
    #camera
    base.cam.setPos(position[0]-15*direction[0], position[1]-15*direction[1], 4)
    base.cam.lookAt(position)

  # update path with centralized multi car planner
  def updatePath(self):
    # get reference multi path
    # call CFS_planner.Plan_trajectory
    # update path for ecery agent
    num_cars = len(self.initAV)
    if self.replanFlag is True:

      MAX_ITER = 20
      min_dist = 1
      num_steps = 20
      multi_path = np.zeros((num_cars, num_steps, 2))
      for i in range(num_cars):
        # multi_path[i] = np.linspace([-6,9], [-6,28], 20)
        multi_path[i] = np.asarray(self.agents[i].getPreview(self.agents[i].getCurrLaneId(),num_steps-1))
      new_path = CFS_planner.Plan_trajectory(MAX_ITER, multi_path, min_dist)

      for i in range(num_cars):
        car_path = np.zeros((num_steps, 2))
        car_path[:, 0] = new_path[2*i : : num_cars*2]
        car_path[:, 1] = new_path[2*i+1 : : num_cars*2]
        self.agents[i].traj = car_path
      self.replanFlag = False

    else:

      for i in range(num_cars):
        # follow last planned trajectory until finished
        if self.agents[i].traj is not None and len(self.agents[i].traj)>2:
          pos = self.agents[i].getState()[0]
          slope = self.agents[i].traj[1] - self.agents[i].traj[0]
          constant = -slope.dot(self.agents[i].traj[0])
          if (slope.dot(self.agents[i].traj[1])+constant)*(slope.dot(pos)+constant) > 0:
              self.agents[i].traj = self.agents[i].traj[1:]
        else:
          self.replanFlag = True

      

  # simulation update per step
  def update(self, task):
    dt = globalClock.getDt()
    # central planner
    self.updatePath()
    for i in range(len(self.initAV)):
        self.vehicles[i].controlInput(self.vehicles[i].agent.doControl())     
    
    #self.vehicles[0].processInput(dt,'forward','reverse','turnLeft','turnRight','brake1')  
    #self.vehicles[1].processInput(dt,'For','Back','Lef','Righ','brake2')    # manual control
    #self.world.doPhysics(dt, 10, 0.008)
    self.world.doPhysics(1.6)

    self.updateCamera() 
    
    return task.cont

  # exit
  def cleanup(self):
    self.world = None
    self.worldNP.removeNode()

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
    

game = Game()
base.run()

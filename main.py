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
import pandas as pd
import math
import time

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

  def read_Holo_data(self):
    # ego = pd.read_csv('Holo_data/ego_sample.csv')
    # obs = pd.read_csv('Holo_data/obs_sample.csv',low_memory=False)
    ego = pd.read_csv('Holo_data/2018-12-04-10-59-21-ego-car-odometry.csv')
    obs = pd.read_csv('Holo_data/2018-12-04-10-59-21-obstacles.csv',low_memory=False)

    ego = ego.rename(index=str, columns={"header.stamp.secs": "s", "header.stamp.nsecs": "ns", "pose.position":"pos", "pose.orientation":"ori"})
    ego.pos = ego.apply(lambda x: list(map(float, x.pos[1:-1].split(','))), axis=1)
    ego.ori = ego.apply(lambda x: list(map(float, x.ori[1:-1].split(','))), axis=1)
    ego.s = list(map(int, ego['s']))
    ego.ns = list(map(int, ego['ns']))
    ego.time = ego.apply(lambda x: x.s - ego.s[0] + x.ns/1e9 , axis=1)
    ego = ego.drop(['s','ns','pose_covariance'],axis=1)
    ego.head()
    ego.pos = ego.pos.apply(np.array)
    ego.ori = ego.ori.apply(np.array)
    ego.pos = ego.pos.apply(lambda x: x - ego.pos[0])

    def f2e(f):
      x,y,z,w = f
      rol = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
      yaw = np.arcsin(2 * (w*y - z*x))
      pit = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
      return [rol, yaw, pit]

    ego.ori = ego.ori.apply(f2e)

    
    obs = obs.rename(index=str, columns={"header.stamp.secs": "s", "header.stamp.nsecs": "ns"})
    obs.s = list(map(int, obs['s']))
    obs.ns = list(map(int, obs['ns']))
    obs.time = obs.apply(lambda x: x.s - obs.s[0] + x.ns/1e9 , axis=1)
    obs.iloc[:,0:15]
    obs = obs.drop(['s','ns'],axis=1)
    obs = obs[6:]

    self.ego = ego
    self.obs = obs
    self.timer = 0

  def __init__(self):
    self.read_Holo_data()

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
    self.initAV=[50,0,0]
    desiredV=40
    basicVehicleNum=20
    self.agents=[]
    self.agents.append(planningAgent(20,5,desiredV,int(math.floor((self.initAV[1]+8)/4)),basicVehicleNum,self.radiu)) # initial agent

    # initial surrounding vehicle
    v1=0
    v2=0
    v3=0
    v4=0
    self.initSV=[]
    
  

    for i in range(20):
      self.initSV.append([1000,0,v1])
      self.agents.append(laneKeepingAgent(20,20,self.initSV[-1][2],int(math.floor((self.initSV[-1][1]+8)/4)))) 
      

    # initial camera
    base.cam.setPos(0, -20, 3)
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

    inputState.watchWithModifiers('forward', 'w')
    inputState.watchWithModifiers('reverse', 's')
    inputState.watchWithModifiers('turnLeft', 'a')
    inputState.watchWithModifiers('turnRight', 'd')
    inputState.watchWithModifiers('brake1', 'x')

    inputState.watchWithModifiers('For', 'i')
    inputState.watchWithModifiers('Back', 'k')
    inputState.watchWithModifiers('Lef', 'j')
    inputState.watchWithModifiers('Righ', 'l')
    inputState.watchWithModifiers('brake2', 'space')
    

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
    base.cam.setPos(position[0]-30*direction[0], position[1]-60*direction[1], 30)
    base.cam.lookAt(position + LVector3f(0, 0, 0))

  # simulation update per step
  def update(self, task):
    dt = globalClock.getDt()
    # self.vehicles[0].controlInput(self.vehicles[0].agent.doControl())  # agent control
    
    # for i in range(len(self.initSV)):
    #     self.vehicles[i+1].controlInput(self.vehicles[i+1].agent.doControl()) 
    
    car = self.vehicles[0].yugoNP.parent
    # print(car.getPos())
    # print(self.ego.pos[self.timer])
    p = self.ego.pos[self.timer]
    p = LPoint3f(p[0], p[1], p[2])
    
    # car.setPos(p)
    car.setH(self.ego.ori[self.timer][1])
    x = self.obs.iloc[self.timer]

    for i in range(0,len(x)//4):
      if not isinstance(x[i*4+1], str):
        for j in range(i+1,20):
          self.vehicles[j].yugoNP.parent.setPos(1000, 0, 0)
        break
      pos = list(map(float,x[i*4+1][1:-1].split(',')))
      shape = list(map(float,x[i*4+3][1:-1].split(',')))
      yaw = shape[-1]

      p = LPoint3f(-pos[1], pos[0], pos[2])
      # print(self.vehicles[i].yugoNP.parent.getPos())
      self.vehicles[i+1].yugoNP.parent.setPos(car.getPos() + p)
      # print(self.vehicles[i].yugoNP.parent.getPos())
      # print('---')
      self.vehicles[i+1].yugoNP.parent.setH(yaw)


    self.timer = self.timer + 1
    
    # time.sleep(1)
    #self.vehicles[0].processInput(dt,'forward','reverse','turnLeft','turnRight','brake1')  
    #self.vehicles[1].processInput(dt,'For','Back','Lef','Righ','brake2')    # manual control
    #self.world.doPhysics(dt, 10, 0.008)
    # self.world.doPhysics(1.6)

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
    self.debugNP.show()

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
    
    self.vehicles.append(basicVehicle(self,[self.initAV[0],self.initAV[1],-0.6],self.initAV[2],length,width,height,axisDis,wheelDis,radius,wheelH)) # [10,0.1,0.5] is vehicle start position
    self.sensors.append(basicSensor(self))  # initial sensor
    self.sensors[0].setVehicle(self.vehicles[0])
    self.vehicles[0].setSensor(self.sensors[0])
    self.vehicles[0].sensor.align()
    self.agents[0].setVehicle(self.vehicles[0])
    self.vehicles[0].setAgent(self.agents[0])

    #Adding laneKeeping vehicles
    for i in range(len(self.initSV)):
        self.vehicles.append(basicVehicle(self,[self.initSV[i][0],self.initSV[i][1],-0.6],self.initSV[i][2],length,width,height,axisDis,wheelDis,radius,wheelH)) # [10,0.1,0.5] is vehicle start position
        self.sensors.append(basicSensor(self))  # initial sensor
        self.sensors[i+1].setVehicle(self.vehicles[i+1])
        self.vehicles[i+1].setSensor(self.sensors[i+1])
        self.vehicles[i+1].sensor.align()
        self.agents[i+1].setVehicle(self.vehicles[i+1])
        self.vehicles[i+1].setAgent(self.agents[i+1])    
    
    '''self.vehicles.append(basicVehicle(self,[50,-6,-0.6],30)) # [10,0.1,0.5] is vehicle start position
    sensor1=basicSensor(self)  # initial sensor
    sensor1.setVehicle(self.vehicles[1])
    self.vehicles[1].setSensor(sensor1)
    self.vehicles[1].sensor.align()
    #agent1=planningAgent(20,5,40,0,1)   # initial agent
    agent1=laneKeepingAgent(20,20,30,1)
    agent1.setVehicle(self.vehicles[1])
    self.vehicles[1].setAgent(agent1)'''
    
    #Surrounding vehicles' speed
    v1=0
    v2=0
    v3=0
    v4=0
    
    '''self.vehicles.append(basicVehicle(self,[60,-6.1,-0.6],v1))
    self.vehicles.append(basicVehicle(self,[100,-6.1,-0.6],v1))
    self.vehicles.append(basicVehicle(self,[155,-6.1,-0.6],v1))
    self.vehicles.append(basicVehicle(self,[215,-6.1,-0.6],v1))
    self.vehicles.append(basicVehicle(self,[270,-6.1,-0.6],v1))
    
    self.vehicles.append(basicVehicle(self,[70,-2,-0.6],v2))
    self.vehicles.append(basicVehicle(self,[115,-2,-0.6],v2))
    self.vehicles.append(basicVehicle(self,[165,-2,-0.6],v2))
    self.vehicles.append(basicVehicle(self,[230,-2,-0.6],v2))
    self.vehicles.append(basicVehicle(self,[300,-2,-0.6],v2))

    self.vehicles.append(basicVehicle(self,[90,2,-0.6],v3))
    self.vehicles.append(basicVehicle(self,[140,2,-0.6],v3))
    self.vehicles.append(basicVehicle(self,[200,2,-0.6],v3))
    self.vehicles.append(basicVehicle(self,[250,2,-0.6],v3))
    self.vehicles.append(basicVehicle(self,[320,2,-0.6],v3))

    self.vehicles.append(basicVehicle(self,[65,6.1,-0.6],v4))
    self.vehicles.append(basicVehicle(self,[130,6.1,-0.6],v4))
    self.vehicles.append(basicVehicle(self,[220,6.1,-0.6],v4))
    self.vehicles.append(basicVehicle(self,[275,6.1,-0.6],v4))
    self.vehicles.append(basicVehicle(self,[350,6.1,-0.6],v4))'''
    

game = Game()
base.run()

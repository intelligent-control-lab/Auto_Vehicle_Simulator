####################
# main.py
# this file visualize ngsim format data from aerial view.
#
# Author: WEI Tianhao
# Copyright: 2019
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

  def read_ngsim_data(self):
    self.df = pd.read_csv('data/sample.csv')
    

  def __init__(self):
    self.read_ngsim_data()

    base.setBackgroundColor(0.1, 0.1, 0.8, 1)
    base.setFrameRateMeter(True)

    self.x_center = 36

    self.timer = self.df.iloc[0].Global_Time
    self.idx = 0

    # road geometry generation
    #road=trail(100,50,40,0.2) # generating a circular lane track
    self.precision=1 # length of a piece of centerLine
    self.texLength=10
    self.laneNum=4
    self.radiu=500
    # road=basicFreeWay(200,self.radiu,5,self.precision,4) # generating a piece of freeway
    road=straightCenter(np.array([self.x_center,0]),math.pi/2,2000,2)  # generating a straight way
    self.segLine=road.getLine() # the centerLine
    self.road=roadGenerator(np.array([self.x_center,-1]),12,8,road.getFollowing(),self.segLine,-1,self.texLength,self.precision) # generate road polygon
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
    # The lane number on texture pic should be consistent with data
    floorTex = loader.loadTexture('maps/6lane.jpg')
    floor = render.attachNewNode(node)
    floor.setTexture(floorTex)
    # floor.flattenStrong()
    
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


    # self.agents.append(planningAgent(20,5,desiredV,int(math.floor((self.initAV[1]+8)/4)),basicVehicleNum,self.radiu)) # initial agent

    # initial surrounding vehicle
    
    self.vehicles_maxload = 100

    # for i in range(self.vehicles_maxload):
    #   self.initSV.append([1000,0,v1])
    #   self.agents.append(laneKeepingAgent(20,20,self.initSV[-1][2],int(math.floor((self.initSV[-1][1]+8)/4)))) 
      

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
    self.accept('f1', self.toggleWireframe)
    self.accept('f2', self.toggleTexture)
    self.accept('f3', self.toggleDebug)
    self.accept('f5', self.doScreenshot)
    
    # Task manager
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____
  def doExit(self):
    self.cleanup()
    sys.exit(1)


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

    base.cam.setPos(self.x_center + 300, 150, 300)
    base.cam.lookAt(self.x_center - 200, 300, -200)

  # simulation update per step
  def update(self, task):
    dt = globalClock.getDt()
    
    cnt = 0
    show_up = set()
    if self.idx >= len(self.df):
        print('finished')
        self.doExit()
    while self.idx < len(self.df):
        
        r = self.df.iloc[self.idx]
        if r.Global_Time != self.timer:
            break
        self.idx = self.idx + 1

        if r.Vehicle_ID not in self.vehicles:
            height=1
            axisDis=2.1
            wheelDis=1.4
            wheelH=0.3
            radius=0.25
            self.vehicles[r.Vehicle_ID] = basicVehicle(self,[0,0,-0.6],0,r.v_length,r.v_Width,height,axisDis,wheelDis,radius,wheelH)

        if r.v_Class == 1:
            print('A moto')
            continue

        show_up.add(r.Vehicle_ID)
        p = LPoint3f(r.Local_X, r.Local_Y, 0)
        self.vehicles[r.Vehicle_ID].yugoNP.parent.setPos(p)
        cnt = cnt+1
        
    del_keys = []
    for j in self.vehicles.keys():
        if j not in show_up:
            del_keys.append(j)
        
    for j in del_keys:
        del self.vehicles[j]

    self.timer = self.timer + 100
    
    time.sleep(0.05)

    self.updateCamera() 
    
    return task.cont

  # exit
  def cleanup(self):
    self.world = None
    

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

    
    self.vehicles=dict()
    
game = Game()
base.run()

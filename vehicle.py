#############
# vehicle.py
# This file contains classes to define a vehicle,
# including the shape, parameters, control execution and states
#
# Author: Jianyu Chen
# 2016
#############

from panda3d.bullet import BulletVehicle
from panda3d.bullet import ZUp
from panda3d.bullet import BulletBoxShape
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import TransformState
from panda3d.core import Point3
from panda3d.bullet import BulletRigidBodyNode
from direct.showbase.InputStateGlobal import inputState
import math
import numpy

# Set the chassis of the vehicle
def setChassis(game,position,velocity,length,width,height):
  shape = BulletBoxShape(Vec3(width/2, length/2, height/2)) #The shape of the chassis
  ts = TransformState.makePos(Point3(0, 0, 0.5))

  np = game.worldNP.attachNewNode(BulletRigidBodyNode('Vehicle'))
  np.node().addShape(shape, ts)
  np.setPos(position)
  np.node().setLinearVelocity(Vec3(0,velocity,0))
  np.node().setMass(1000.0) # The mass of the vehicle
  np.node().setDeactivationEnabled(False)

  game.world.attachRigidBody(np.node())
  return np

class basicVehicle(BulletVehicle):
  def __init__(self,game,pos,vel,length,width,height,axisDis,wheelDis,radius,wheelH):
    self.initCordPos=numpy.array([pos[0],pos[1]])
    '''self.length=length
    self.width=width
    self.height=height
    self.axisDis=axisDis
    self.wheelDis=wheelDis
    self.radius=radius
    self.wheelH=wheelH'''
    line=game.segLine
    lineLen=0
    prevPoint=line[0]
    self.initNum=0
    for point in line:
      norm=numpy.linalg.norm(point-prevPoint)
      if pos[0]>=lineLen and pos[0]<lineLen+norm:
        ldirec=(point-prevPoint)/norm
        rdirec=numpy.array([ldirec[1],-ldirec[0]])
        post=prevPoint+(pos[0]-lineLen)*ldirec+rdirec*pos[1]
        self.initPos=Vec3(post[0],post[1],pos[2])
        break
      lineLen=lineLen+norm
      prevPoint=point
      self.initNum=self.initNum+1
    chassis=setChassis(game,self.initPos,vel,length,width,height)
    BulletVehicle.__init__(self,game.world,chassis.node())
    self.setCoordinateSystem(ZUp)
    game.world.attachVehicle(self)
    #self.yugoNP = loader.loadModel('models/yugo/yugo.egg')
    self.yugoNP = loader.loadModel('models/car.egg')
    self.yugoNP.reparentTo(chassis)

    # Right front wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( wheelDis/2,  axisDis/2, wheelH), True, np, radius)

    # Left front wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-wheelDis/2,  axisDis/2, wheelH), True, np, radius)

    # Right rear wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( wheelDis/2, -axisDis/2, wheelH), False, np, radius)

    # Left rear wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-wheelDis/2, -axisDis/2, wheelH), False, np, radius)

    # Steering info
    self.steering = 0.0            # degree
    self.steeringClamp = 45.0      # degree
    self.steeringIncrement = 10.0  # degree per second

  def addWheel(self, pos, front, np, radius):
    wheel = self.createWheel()

    wheel.setNode(np.node())
    wheel.setChassisConnectionPointCs(pos)
    wheel.setFrontWheel(front)

    wheel.setWheelDirectionCs(Vec3(0, 0, -1))
    wheel.setWheelAxleCs(Vec3(1, 0, 0))
    wheel.setWheelRadius(radius) # the radius of the wheel
    wheel.setMaxSuspensionTravelCm(40.0)

    wheel.setSuspensionStiffness(40.0)
    wheel.setWheelsDampingRelaxation(2.3)
    wheel.setWheelsDampingCompression(4.4)
    wheel.setFrictionSlip(100.0);
    wheel.setRollInfluence(0.1)

  # process the input from the keyboard
  def processInput(self, dt, up, back, left, right, brake):
    engineForce = 0.0
    brakeForce = 0.0
    #direction=self.getForwardVector()
    if inputState.isSet(up):
      engineForce = 2000.0
      brakeForce = 0.0
    
    if inputState.isSet(back):
      engineForce = -1000.0
      brakeForce = 0.0
    
    if inputState.isSet(brake):
      engineForce = 0.0
      brakeForce = 1000.0
      
    if inputState.isSet(left):
      self.steering += dt * self.steeringIncrement
      self.steering = min(self.steering, self.steeringClamp)
    elif inputState.isSet(right):
      self.steering -= dt * self.steeringIncrement
      self.steering = max(self.steering, -self.steeringClamp)
    else:
      self.steering=self.steering*0.7

    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)

    
  # get the vehicle 3D position
  def getPosVector(self):
    return self.getChassis().getTransform().getPos()

  # get the vehicle x-y position
  def getPos(self):
    return numpy.array([self.getPosVector()[0],self.getPosVector()[1]])

  # get the vehicle heading
  def getDirection(self):
    return self.getForwardVector()

  # get |v|
  def getVelocity(self):
    V=numpy.array([self.getChassis().getLinearVelocity()[0],self.getChassis().getLinearVelocity()[1]])
    return numpy.linalg.norm(V)

  # get v(in x-y plane)
  def getVelocityVector(self):
    return numpy.array([self.getChassis().getLinearVelocity()[0],self.getChassis().getLinearVelocity()[1]])

  # get the yaw rate
  def getAngleVelocity(self):
    return self.getChassis().getAngularVelocity()[2]

  # attach sensor to the vehicle
  def setSensor(self,sensor):
    self.sensor=sensor

  # attach agent to the vehicle
  def setAgent(self,agent):
    self.agent=agent

  # execute the control input given by agent
  def controlInput(self,agentInput): #the input acc is m/s
    engineForce = 0.0
    brakeForce = 0.0
    accGain=500

    engineForce=agentInput[0]*accGain #transfer to engineForce
    self.steering=agentInput[1] #In degree not rad
    brakeForce=agentInput[2]
    #print(brakeForce)

    if engineForce>5.5*accGain:
        engineForce=5.5*accGain
    if engineForce<-35*accGain:
        engineForce=-35*accGain   
        
    steeringLimit=45

    if self.steering>steeringLimit:
      self.steering=steeringLimit
    if self.steering<-steeringLimit:
      self.steering=-steeringLimit
    
    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)

class surroundingVehicle(BulletVehicle):
  def __init__(self,game,pos,vel,length,width,height,axisDis,wheelDis,radius,wheelH):
    self.initCordPos=numpy.array([pos[0],pos[1]])
    '''self.length=length
    self.width=width
    self.height=height
    self.axisDis=axisDis
    self.wheelDis=wheelDis
    self.radius=radius
    self.wheelH=wheelH'''
    line=game.segLine
    lineLen=0
    prevPoint=line[0]
    self.initNum=0
    for point in line:
      norm=numpy.linalg.norm(point-prevPoint)
      if pos[0]>=lineLen and pos[0]<lineLen+norm:
        ldirec=(point-prevPoint)/norm
        rdirec=numpy.array([ldirec[1],-ldirec[0]])
        post=prevPoint+(pos[0]-lineLen)*ldirec+rdirec*pos[1]
        self.initPos=Vec3(post[0],post[1],pos[2])
        break
      lineLen=lineLen+norm
      prevPoint=point
      self.initNum=self.initNum+1
    chassis=setChassis(game,self.initPos,vel,length,width,height)
    BulletVehicle.__init__(self,game.world,chassis.node())
    self.setCoordinateSystem(ZUp)
    game.world.attachVehicle(self)
    self.yugoNP = loader.loadModel('models/yugo/yugo.egg')
    #self.yugoNP = loader.loadModel('models/car.egg')
    self.yugoNP.reparentTo(chassis)

    # Right front wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( wheelDis/2,  axisDis/2, wheelH), True, np, radius)

    # Left front wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-wheelDis/2,  axisDis/2, wheelH), True, np, radius)

    # Right rear wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( wheelDis/2, -axisDis/2, wheelH), False, np, radius)

    # Left rear wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-wheelDis/2, -axisDis/2, wheelH), False, np, radius)

    # Steering info
    self.steering = 0.0            # degree
    self.steeringClamp = 45.0      # degree
    self.steeringIncrement = 10.0  # degree per second

  def addWheel(self, pos, front, np, radius):
    wheel = self.createWheel()

    wheel.setNode(np.node())
    wheel.setChassisConnectionPointCs(pos)
    wheel.setFrontWheel(front)

    wheel.setWheelDirectionCs(Vec3(0, 0, -1))
    wheel.setWheelAxleCs(Vec3(1, 0, 0))
    wheel.setWheelRadius(radius) # the radius of the wheel
    wheel.setMaxSuspensionTravelCm(40.0)

    wheel.setSuspensionStiffness(40.0)
    wheel.setWheelsDampingRelaxation(2.3)
    wheel.setWheelsDampingCompression(4.4)
    wheel.setFrictionSlip(100.0);
    wheel.setRollInfluence(0.1)

  # process the input from the keyboard
  def processInput(self, dt, up, back, left, right, brake):
    engineForce = 0.0
    brakeForce = 0.0
    #direction=self.getForwardVector()
    if inputState.isSet(up):
      engineForce = 2000.0
      brakeForce = 0.0
    
    if inputState.isSet(back):
      engineForce = -1000.0
      brakeForce = 0.0
    
    if inputState.isSet(brake):
      engineForce = 0.0
      brakeForce = 1000.0
      
    if inputState.isSet(left):
      self.steering += dt * self.steeringIncrement
      self.steering = min(self.steering, self.steeringClamp)
    elif inputState.isSet(right):
      self.steering -= dt * self.steeringIncrement
      self.steering = max(self.steering, -self.steeringClamp)
    else:
      self.steering=self.steering*0.7

    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)

    
  # get the vehicle 3D position
  def getPosVector(self):
    return self.getChassis().getTransform().getPos()

  # get the vehicle x-y position
  def getPos(self):
    return numpy.array([self.getPosVector()[0],self.getPosVector()[1]])

  # get the vehicle heading
  def getDirection(self):
    return self.getForwardVector()

  # get |v|
  def getVelocity(self):
    V=numpy.array([self.getChassis().getLinearVelocity()[0],self.getChassis().getLinearVelocity()[1]])
    return numpy.linalg.norm(V)

  # get v(in x-y plane)
  def getVelocityVector(self):
    return numpy.array([self.getChassis().getLinearVelocity()[0],self.getChassis().getLinearVelocity()[1]])

  # get the yaw rate
  def getAngleVelocity(self):
    return self.getChassis().getAngularVelocity()[2]

  # attach sensor to the vehicle
  def setSensor(self,sensor):
    self.sensor=sensor

  # attach agent to the vehicle
  def setAgent(self,agent):
    self.agent=agent

  # execute the control input given by agent
  def controlInput(self,agentInput): #the input acc is m/s
    engineForce = 0.0
    brakeForce = 0.0
    accGain=500

    engineForce=agentInput[0]*accGain #transfer to engineForce
    self.steering=agentInput[1] #In degree not rad
    brakeForce=agentInput[2]
    #print(brakeForce)

    if engineForce>5.5*accGain:
        engineForce=5.5*accGain
    if engineForce<-35*accGain:
        engineForce=-35*accGain   
        
    steeringLimit=45

    if self.steering>steeringLimit:
      self.steering=steeringLimit
    if self.steering<-steeringLimit:
      self.steering=-steeringLimit
    
    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)
    



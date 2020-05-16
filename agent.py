###################
# agent.py
# this file contains classes to define an agent,
# each agent generates control input to sent to the vehicle
#
# Authors: Changliu Liu, Jianyu Chen
# Copyright: 2016
###################
from __future__ import division
from direct.stdpy.file import *
import math
import numpy as np
from cvxopt import matrix, solvers
import utility
import optimization as opt
import copy
solvers.options['feastol'] = 1e-10

# from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import time

# this is a basic agent structure that generate zero input
class basicAgent:
    def __init__(self):
        return

    def setVehicle(self,vehicle):
        self.vehicle=vehicle

    # Get global position (x,y)
    def getPos(self):
        return self.vehicle.sensor.getSelfPos()

    # Get current velocity v
    def getVelocity(self):
        return self.vehicle.sensor.getVelocity()

    def getVelocityVector(self):
        return self.vehicle.getVelocityVector()

    # Get direction of the vehilce in vector form
    def getDirection(self):
        return self.vehicle.sensor.getSelfAngle()

    # Get the state of the vehicle: x,y,v,theta
    def getState(self):
        # state = self.getPos()
        # state.append(self.getVelocity())
        # state.append(utility.vec2ang(self.getDirection()))
        return [self.getPos(), self.getVelocity(), self.getDirection()]

    # Get distance to the lane center (lane ID=lf)
    def getDis(self,lf=1):
        return self.vehicle.sensor.getCordPos(lf)[0]

    # Get the angle between the vehicle and the road
    def getAngle(self):
        return self.vehicle.sensor.getCordAngle()

    # Get angular velocity of the vehicle theta dot
    def getAngularVelocity(self):
        return self.vehicle.getAngleVelocity()

    def doControl(self):
        return [0,0,0]

# the laneKeepingAgent keeps driving in the centerline
class laneKeepingAgent(basicAgent):
    def __init__(self,vGain=20,thetaGain=20,desiredV=35,laneId=0):
        self.vGain = vGain
        self.thetaGain = thetaGain
        self.desiredV = desiredV
        self.targetLane = laneId

    def setTargetLane(self,laneId):
        self.targetLane = laneId

    # Feedback control is based on the angle difference and position difference
    def getFeedbackControl(self,diffAngle,diffPos,diffPosV):
        
        acceleration=self.vGain*(self.desiredV*math.cos(self.getAngle())-self.getVelocity())
        steer=-self.thetaGain*diffAngle/(self.getVelocity()+1)-3*self.vehicle.getAngleVelocity()-5*diffPos-5*diffPosV
        return [acceleration,steer]

    def doControl(self):
        diffPosV=self.vehicle.sensor.getCordVelocity((self.vehicle.sensor.getLineInRange(0,2,self.targetLane)))
        fb=self.getFeedbackControl(self.getAngle(),self.getDis(self.targetLane),diffPosV)
        return [fb[0],fb[1],0]

# the previewAgent adds preview control to the laneKeepingAgent
class previewAgent(laneKeepingAgent):
    def __init__(self,vGain=5000,thetaGain=20,desiredV=15,laneId=0,ffGain=1000):
        self.vGain = vGain
        self.thetaGain = thetaGain
        self.ffGain=ffGain
        self.desiredV = desiredV
        self.targetLane = laneId
        self.ts = 1.0/15

    def getPreview(self,laneId=0,length=20):
        return self.vehicle.sensor.getLineInRange(0,length,laneId)

    # Feedforward control based on future trajectory
    def getFeedforwardControl(self,futureTraj):
        # Steering
        vehicleDirection = self.getDirection()
        displacement = []

        for index in range(len(futureTraj)-1):
            trajDirection = [futureTraj[index+1][0]-futureTraj[0][0],futureTraj[index+1][1]-futureTraj[0][1]]
            vector = utility.perpLength(trajDirection,vehicleDirection)
            length = np.linalg.norm(vector)
            #displacement.append((-trajDirection[0]*vehicleDirection[1]+trajDirection[1]*vehicleDirection[0])/(vehicleDirection[0]**2+vehicleDirection[1]**2)*vector)
            displacement.append(length*vector)

        steerV = 0
        for index in range(len(displacement)):
            steerV += displacement[index]/(index + 1)/(self.getVelocity()+1)
        steerV =  steerV / (index + 1)


        # Accelaration
        vehicleVelocity = self.getVelocity()
        deltav = []

        for index in range(len(futureTraj)-1):
            refVel = np.linalg.norm(np.array([futureTraj[index+1][0]-futureTraj[index][0],futureTraj[index+1][1]-futureTraj[index][1]]))
            deltav.append(refVel - vehicleVelocity)

        accelaration = 0
        for index in range(len(displacement)):
            accelaration += deltav[index]/(index + 1)
        accelaration = accelaration / (index+1)
        #self.desiredV = np.linalg.norm(np.array([(futureTraj[-1][0]-futureTraj[0][0]),(futureTraj[-1][1]-futureTraj[0][1])]))/index/self.ts

        return [accelaration,steerV]

    # Preview control for lane id = lf
    def previewController(self,laneId=0):
        futureTraj = self.getPreview(laneId)
        diffPosV=self.vehicle.sensor.getCordVelocity(futureTraj)
        ff = self.getFeedforwardControl(futureTraj)
        fb = self.getFeedbackControl(self.getAngle(),self.getDis(laneId),diffPosV)
        ff=ff*self.ffGain
        acceleration = ff[0]+fb[0]
        steerV = ff[1]+fb[1]
        return [acceleration,steerV,0]

    def doControl(self):
        return self.previewController(self.targetLane)

# the autoBrakeAgent brakes when encontering obstacles in front
class autoBrakeAgent(previewAgent):
    def __init__(self,vGain=50,thetaGain=20,desiredV=15,laneId=0,ffGain=1000,headway=20):
        self.vGain=vGain
        self.thetaGain=thetaGain
        self.desiredV=desiredV
        self.safeHeadway = headway

        self.ffGain=ffGain
        self.targetLane = laneId

    # Returns relative position vector and relative velocity vector
    def getSurroundVehicleRelateState(self,num):
        return self.vehicle.sensor.getSurroundVehicleRelateState(num)

    # Returns position vector and velocity vector
    def getSurroundVehicleState(self,num):
        return self.vehicle.sensor.getSurroundVehicleState(num)

    def getHeadway(self):
        vehicleDirection = self.vehicle.sensor.getSelfAngle()
        [relateX,relateV]=self.getSurroundVehicleRelateState(1)
        if np.linalg.norm(np.cross(vehicleDirection,relateX))<2:
            dis = np.dot(vehicleDirection,relateX)
            if dis<0:
                return self.safeHeadway
            else:
                return dis
        else:
            return self.safeHeadway

    def autoBrake(self,laneId=0):
        headway = self.getHeadway()
        if headway < self.safeHeadway:

            if headway < self.safeHeadway/2:
                #print('brake')
                return [0,0,100/(headway+0.1)]
            else:
                return [-50/(headway+0.1),self.previewController(laneId)[1]/(self.safeHeadway/headway)**0.5,0]
        else:
            return self.previewController(laneId)

    def doControl(self,laneId=0):
        return self.autoBrake(laneId)

# the planningAgent use safety controller to achieve collision avoidance
class planningAgent(autoBrakeAgent):
    def __init__(self,vGain=50,thetaGain=20,desiredV=25,laneId=0,num=1,radiu=500,headway=20):
        self.vGain=vGain
        self.thetaGain=thetaGain
        self.desiredV=desiredV
        self.safeHeadway = headway
        self.traj = [[0,0]]
        self.numSurrounding = num # number of surrounding vehicles
        self.h=2
        self.dmin = 10
        self.ts = 1.00/15
        self.targetLane=laneId
        self.changeFlag=0
        self.phiFlag=0
        self.frontObs=0
        self.radiu=radiu
        self.Range=15
        self.alpha=50
        self.ita=2
        self.changeThres=1.3
        self.previousInput=[0,0]
        self.timeStep=0
        safetyCommand = open('safetyCommand.txt', 'w')
        vProf = open('velocity.txt', 'w')
        dProf = open('distance.txt', 'w')

    # The Efficiency controller
    def getTrajectory(self):
        #return self.getPreview(1,25)

        horizon = 20
        refTraj = []
        if len(self.traj)>1:
            refTraj = self.traj[:]

        preTraj = self.getPreview(self.getCurrLaneId(),horizon)
        #print(self.getCurrLaneId())
        if len(self.traj)>1:
            i = len(self.traj)
        else:
            i = len(self.traj)-1

        while i < horizon:
            i = i+1
            refTraj.append(preTraj[i])
        #print(len(refTraj))

        x0 = self.getPos()

        # get trajectory of surrounding vehicles
        obs = []
        for i in range(self.numSurrounding):
            obs.append(self.getPrediction(i,horizon))
        #print(obs)
        traj = opt.CFS_FirstOrder(x0,refTraj,obs,horizon,self.ts)
        return traj


    # Get lateral distance to a trajectory; return (dis, angle, index)
    def getDisAngle(self,x0,theta,traj):
        l1 = np.linalg.norm(utility.substruct(traj[0],x0))
        for index in range(len(traj)-1):
            l2 = np.linalg.norm(utility.substruct(traj[index+1],x0))
            direction = utility.substruct(traj[index+1],traj[index])
            angle = utility.vec2ang(direction)

            if l1**2+l2**2 < np.linalg.norm(direction)**2 or l1 < l2:
                return (utility.perpLength(utility.substruct(x0,traj[index]),direction),(theta-angle+math.pi) % (2*math.pi)-math.pi,index)


    # Overwrite the preview controller in Preview Agent
    def previewController(self):

        '''h = self.h

        if len(self.traj)<h:
            self.traj = list(self.getTrajectory())'''

        self.traj = self.getPreview(self.targetLane,25)
        futureTraj = self.traj
        #print(futureTraj)

        ff = [0,0]#self.getFeedforwardControl(futureTraj)
        
        #dis, angle, index = self.getDisAngle(self.getPos(),utility.vec2ang(self.getDirection()),futureTraj)
        diffPosV=self.vehicle.sensor.getCordVelocity(futureTraj)
        fb = self.getFeedbackControl(self.getAngle(),self.getDis(self.targetLane),diffPosV)
        #print(dis-self.getDis(),angle-self.getAngle())

        '''i = 0
        while i <= index:
            self.traj.pop(0)
            i += 1'''

        acceleration = ff[0]+fb[0]
        steerV = ff[1]+fb[1]

        steeringLimit=45

        if steerV>steeringLimit:
          steerV=steeringLimit
        if steerV<-steeringLimit:
          steerV=-steeringLimit

        return [acceleration,steerV,0]

    def getCurrLaneId(self):
        '''pos=self.getPos()
        if pos[0]<-3.6:
            return 0
        if pos[0]<0:
            return 1
        if pos[0]<3.6:
            return 2
        return 3'''

        dev=-self.vehicle.sensor.getCordPos(0)[0]-6
        if dev<-4:
            return 0
        if dev<0:
            return 1
        if dev<4:
            return 2
        return 3 

    def getSurrVehicleLaneId(self,num):
        '''x=self.getSurroundVehicleState(num)[0][0]
        if x<-3.6:
            return 0
        if x<0:
            return 1
        if x<3.6:
            return 2
        return 3'''
        dev=-self.vehicle.sensor.getSurroundVehicle(num).sensor.getCordPos(0)[0]-6
        if dev<-4:
            return 0
        if dev<0:
            return 1
        if dev<4:
            return 2
        return 3 


    def getLaneDirection(self):
        futureTraj = self.getPreview(1,2)
        laneDirection = np.array([futureTraj[1][0]-futureTraj[0][0],futureTraj[1][1]-futureTraj[0][1]])
        laneDirection = laneDirection/np.linalg.norm(laneDirection)
        return laneDirection

    # Prediction of the future trajectory of surrounding vehicles
    # Here we use constant speed model
    def getPrediction(self,id,horizon):
        [x0,v0] = self.getSurroundVehicleState(id)

        traj = np.zeros((horizon, 2))
        for i in range(horizon):
            traj[i] = x0+v0*self.ts*(i+1)*0.1
        #print(traj)
        return traj

    def changeTargetLane(self):
        self.targetLane=self.getCurrLaneId()

    def safetyController(self):
        self.frontObs=0
        Lstack, Sstack = [], []
        velocity = self.getVelocity()
        velocityVec=self.getVelocityVector()
        vehicleDirection = self.vehicle.sensor.getSelfAngle()
        pos=self.getPos()
        laneDirection =  self.getLaneDirection()
        currLaneObsId=[]
        for index in range(self.numSurrounding):
            if self.getSurrVehicleLaneId(index+1)==self.targetLane:
                currLaneObsId.append(index)
        d=1000
        for index in currLaneObsId:
            [relateX,relateV]=self.getSurroundVehicleRelateState(index+1)
            if np.linalg.norm(relateX)<d:
                d=np.linalg.norm(relateX)
                obs=index

        #the Time
        self.timeStep=self.timeStep+1
        time=self.timeStep/60
        
        #write dProf
        dProf = open('distance.txt', 'a')
        dProf.write(str(time)+'\t'+str(d)+'\n')
        
        currLaneObsId=[obs]
        #print currLaneObsId
        self.phiFlag=0
        for index in currLaneObsId:

            [Xj,Vj] = self.getSurroundVehicleState(index+1)
            [relateXj,relateVj]=self.getSurroundVehicleRelateState(index+1)
            x0=pos[0]
            y0=pos[1]
            v0=velocity
            x0V=velocityVec[0]
            y0V=velocityVec[1]
            xj=Xj[0]
            yj=Xj[1]
            xjV=Vj[0]
            yjV=Vj[1]
            vj=np.sqrt(xjV**2+yjV**2)
            dx=relateXj[0]
            dy=relateXj[1]
            dxV=relateVj[0]
            dyV=relateVj[1]
            d=np.sqrt(dx**2+dy**2)
            dV=(relateXj[0]*relateVj[0]+relateXj[1]*relateVj[1])/d
            phix0=2*dx-self.alpha*(dx*(dxV*dx+dyV*dy)/d**3-dxV/d)
            phixj=-2*dx+self.alpha*(dx*(dxV*dx+dyV*dy)/d**3-dxV/d)
            phiy0=2*dy-self.alpha*(dy*(dxV*dx+dyV*dy)/d**3-dyV/d)
            phiyj=-2*dy+self.alpha*(dy*(dxV*dx+dyV*dy)/d**3-dyV/d) 
            phiv0=self.alpha*(x0V*dx+y0V*dy)/(v0*d)
            phivj=-self.alpha*(xjV*dx+yjV*dy)/(vj*d)    
            phitheta0=-self.alpha*(y0V*dx-x0V*dy)/d
            phithetaj=self.alpha*(yjV*dx-xjV*dy)/d 
            L=[phiv0,phitheta0]  
            phi=self.dmin**2-d**2-self.alpha*dV
            S=-phi-(phixj*xjV+phiyj*yjV)-(phix0*x0V+phiy0*y0V) 

            
            if phi>0 and d<self.Range:
                Lstack.append(L)
                Sstack.append(S)
                self.phiFlag=1
                self.changeFlag=1
                self.frontObs=1

        if self.changeFlag==1:
            '''if self.getCurrLaneId()!=self.targetLane:
                self.changeTargetLane()
                self.changeFlag=0'''
            if -self.vehicle.sensor.getCordPos(self.getCurrLaneId())[0]>self.changeThres:
                self.targetLane=self.getCurrLaneId()+1
            if -self.vehicle.sensor.getCordPos(self.getCurrLaneId())[0]<-self.changeThres:
                self.targetLane=self.getCurrLaneId()-1
            self.changeFlag=0

        if self.getCurrLaneId()==0:
            if self.frontObs==1:
                D=10
            else:
                D=2
            alpha=2
            thetaRel=self.vehicle.sensor.getCordAngle()
            sinTheRel=math.sin(thetaRel)
            cosTheRel=math.cos(thetaRel)
            x=-self.vehicle.sensor.getCordPos(0)[0]
            v=velocity
            r=self.radiu
            L=[alpha*sinTheRel,alpha*v*cosTheRel]
            w=4
            phi=D-w**2/4-x**2-w*x+alpha*v*sinTheRel
            #S=-5-(2*x+w)*v*sinTheRel+alpha*v**2/r*(1-cosTheRel)   # for curve lane
            S=-5-(2*x+w)*v*sinTheRel  # for straight lane

            if phi>0:
                self.phiFlag=1
                Lstack.append(L)
                Sstack.append(S)

        if self.getCurrLaneId()==3:
            if self.frontObs==1:
                D=10
            else:
                D=2
            alpha=2
            thetaRel=self.vehicle.sensor.getCordAngle()
            sinTheRel=math.sin(thetaRel)
            cosTheRel=math.cos(thetaRel)
            x=-self.vehicle.sensor.getCordPos(3)[0]
            v=velocity
            r=self.radiu
            L=[-alpha*sinTheRel,-alpha*v*cosTheRel]
            w=4
            phi=D-w**2/4-x**2+w*x-alpha*v*sinTheRel
            #S=-5-(2*x+w)*v*sinTheRel+alpha*v**2/r*(sinTheRel+1)   # for curve lane
            S=-5-(2*x-w)*v*sinTheRel  # for straight lane
            #print phi
            
            if phi>0:
                self.phiFlag=1
                Lstack.append(L)
                Sstack.append(S)

        refInput = self.previewController()
        ang=refInput[1]*np.pi/180 #transfer to rad
        refU=[refInput[0],np.tan(ang)*velocity/2.1] #transfer from steer to thetaV
        if refU[1]>0.5:
            refU[1]=0.5
        if refU[1]<-0.5:
            refU[1]=-0.5
        
        # The saturation for stability
        Lstack.append([0, 1])
        Sstack.append(0.5)
        Lstack.append([0, -1])
        Sstack.append(0.5)

        vdot0=self.previousInput[0]
        thetadot0=self.previousInput[1]
        dvMax=5
        dthetaMax=0.3
        dthetaMax1=0.3
        a=10
        b=1
        c=0.1
        d=0.1
        
        if self.phiFlag==1:
            Lcheck=copy.copy(Lstack)
            Lcheck.append([1,0])
            Lcheck.append([-1,0])
            Lcheck.append([0,1])
            Lcheck.append([0,-1])
            Scheck=copy.copy(Sstack)
            Scheck.append(vdot0+dvMax)
            Scheck.append(-vdot0+dvMax)
            Scheck.append(thetadot0+dthetaMax)
            Scheck.append(-thetadot0+dthetaMax)
            Lcheck=matrix(Lcheck,(2,len(Lcheck)),'d')
            Lcheck=Lcheck.trans()
            Scheck=matrix(Scheck,(len(Scheck),1),'d')
            Q=matrix([5, 0, 0, 5],(2,2),'d')
            p = matrix(refU[0:2],(2,1),'d')
            p=Q*p
            # Check if Us and Uf intersects
            '''c=matrix([1,1],(2,1),'d')
            sol = solvers.conelp(c, Lcheck, Scheck)
            if sol['status']=='optimal':
                sol = solvers.qp(Q,-p, Lcheck,Scheck)
                newU=sol['x']
            elif sol['status']=='primal infeasible' or sol['status']=='unknown':
                z=[]
                for j in range(len(Lstack)):
                    z.append(0)
                    z.append(0)
                z=matrix(z,(len(Lstack),2),'d')
                Lstack.append([0,0])
                Lstack.append([0,0])
                Lstack.append([0,0])
                Lstack.append([0,0])
                Lstack=matrix(Lstack,(2,len(Lstack)),'d')
                Lstack = Lstack.trans()
                Lplus=matrix([1,-1,0,0,0,0,1,-1],(4,2),'d')
                Lfull=matrix([[Lstack],[matrix([z,Lplus])]])
                Q=matrix([a, 0, -a, 0,0,b,0,-b,-a,0,a,0,0,-b,0,b],(4,4),'d')
                q=matrix([0,0,0,0],(4,1),'d')
                sol = solvers.qp(Q,q, Lfull,Scheck)
                newU=sol['x'][0:2]'''

            z=[]
            for j in range(len(Lstack)):
                z.append(0)
                z.append(0)
            z=matrix(z,(len(Lstack),2),'d')
            Lstack.append([0,0])
            Lstack.append([0,0])
            Lstack.append([0,0])
            Lstack.append([0,0])
            Lstack=matrix(Lstack,(2,len(Lstack)),'d')
            Lstack = Lstack.trans()
            Lplus=matrix([1,-1,0,0,0,0,1,-1],(4,2),'d')
            Lfull=matrix([[Lstack],[matrix([z,Lplus])]])
            #Q=matrix([a, 0, -a, 0,0,b,0,-b,-a,0,a,0,0,-b,0,b],(4,4),'d')
            Q=matrix([a, 0, -a, 0,0,b,0,-b,-a,0,a+c,0,0,-b,0,b+d],(4,4),'d')
            #q=matrix([0,0,0,0],(4,1),'d')
            q=matrix([0,0,-2*c*refU[0],-2*d*refU[1]],(4,1),'d')
            sol = solvers.qp(Q,q, Lfull,Scheck)
            newU=sol['x'][2:4]
            dis=matrix(sol['x'],(4,1),'d').trans()*Q*matrix(sol['x'],(4,1),'d')
                
        else:
            Lsafe=matrix([1,-1,0,0,0,0,1,-1],(4,2),'d')
            Ssafe=matrix([vdot0+dvMax,-vdot0+dvMax,thetadot0+dthetaMax1,-thetadot0+dthetaMax1],(4,1),'d')
            #Ssafe=matrix([vdot0+dvMax,-vdot0+dvMax,0.00002,0.00002],(4,1),'d')
            Q=matrix([5, 0, 0, 5],(2,2),'d')
            p = matrix(refU[0:2],(2,1),'d')
            p=Q*p
            sol = solvers.qp(Q,-p, Lsafe,Ssafe)
            newU=sol['x']
            #newU=refU[0:2]
            #print refU[0:2]

        '''Lstack = matrix(Lstack,(2,len(Lstack)),'d')
        Lstack = Lstack.trans()
        Sstack = matrix(Sstack,(len(Sstack),1),'d')

        #Q = matrix([2, 1, 1, 1000],(2,2),'d')
        Q = matrix([5, 0, 0, 1000],(2,2),'d')
        p = matrix(refU[0:2],(2,1),'d')
        p = Q*p
        #sol = solvers.qp(Q,-p)#, Lstack,Sstack)
        sol = solvers.qp(Q,-p, Lstack,Sstack)
        newU = sol['x']'''
        #print refU
        newInput=copy.copy(newU)

        #check safety controller enabling
        contrEnab=0
        if abs(newInput[0]-refU[0])>0.01 and abs(newInput[1]-refU[1])>0.01:
            contrEnab=1
        #get velocity vector
        latV=self.vehicle.sensor.getCordVelocity(self.getPreview(self.targetLane,1))
        longiV=self.vehicle.sensor.getCordLongiVelocity(self.getPreview(self.targetLane,1))
        
        safetyCommand = open('safetyCommand.txt', 'a')
        safetyCommand.write(str(time)+'\t'+str(contrEnab)+'\n')
        vProf = open('velocity.txt', 'a')
        vProf.write(str(time)+'\t'+str(latV)+'\t'+str(longiV)+'\n')
        
        newInput[1]=np.arctan(newInput[1]/velocity*2.1) #transfer from thetaV to steer
        newInput[1]=newInput[1]*180/np.pi #transfer to degree
        self.previousInput=copy.copy(newU)
        
        
        return [newInput[0],newInput[1],0]

    def doControl(self,lf=1):
        #print(self.previewController())
        #print(self.getVelocity())
        return self.safetyController()

class cfsAgent(autoBrakeAgent):
    _set_timer = False
    _draw_traj = True
    def __init__(self,vGain=50,thetaGain=20,desiredV=25,laneId=0,ffGain=1000,headway=20, numSurr=0):
        super().__init__(vGain,thetaGain,desiredV,laneId,ffGain,headway)
        self.numSurrounding = numSurr
        # for drawing trajectory
        if self._draw_traj:
            lines = LineSegs()
            trajNode = lines.create()
            self.trajNp = NodePath(trajNode)
        self.traj = None
        self.xrec = None

    def getCurrLaneId(self):
        dev=-self.vehicle.sensor.getCordPos(0)[0]-6
        if dev<-4:
            return 0
        if dev<0:
            return 1
        if dev<4:
            return 2
        return 3 

    def getSurrVehicleLaneId(self,num):
        dev=-self.vehicle.sensor.getSurroundVehicle(num).sensor.getCordPos(0)[0]-6
        if dev<-4:
            return 0
        if dev<0:
            return 1
        if dev<4:
            return 2
        return 3 

    def drawTrajectory(self, traj, lines, z):
        for i in range(len(traj)-1):            
            lines.moveTo(traj[i][0],traj[i][1],z)
            lines.drawTo(traj[i+1][0],traj[i+1][1],z)
        
    def previewController(self):
        self.traj = self.CFSTrajectory()
        futureTraj = self.traj

        ff = [0,0]
        diffPosV = self.vehicle.sensor.getCordVelocity(futureTraj)
        fb = self.getFeedbackControl(self.getAngle(),self.getDis(self.targetLane),diffPosV)

        acceleration = ff[0] + fb[0]
        steerV = ff[1] + fb[1]
        steeringLimit = 45

        if steerV > steeringLimit:
          steerV = steeringLimit
        if steerV < -steeringLimit:
          steerV = -steeringLimit

        return [acceleration,steerV,0]

    def CFSTrajectory(self):
        if self._set_timer:
            start_time = time.perf_counter()
        if self._draw_traj:
            self.trajNp.removeNode()    # panda3d draw
            lines = LineSegs()    # panda3d draw

        # Draw reference trajectory
        ego_state = self.getState()
        pos = ego_state[0]
        if self.traj is None:
            self.xrec = pos
        else:
            self.xrec = self.traj[0]
        dist = 18
        preTraj = self.getPreview(0,dist)
        preTraj = preTraj[1:]
        
        # Detect obstacles
        obstacles = []
        trapezoid_orientation = []
        
        for index in range(self.numSurrounding):
            [relateX,relateV] = self.getSurroundVehicleRelateState(index+1)
            if np.linalg.norm(relateX) < dist:
                [X,V] = self.getSurroundVehicleState(index+1)
                if self.getSurrVehicleLaneId(index+1) == 0:
                    trapezoid_orientation.append(0)
                else:
                    trapezoid_orientation.append(1)
                normV = np.linalg.norm(V)
                V /= normV
                obstacles.append([X, V])

                # Draw Obstacles
                '''
                if self._draw_traj:
                    vh_l = 2.8 + 1.0    # in convex_hull_2d
                    vh_w = 1.2 + 0.6
                    a = vh_l / 2
                    b = vh_w / 2
                    d = np.tan(1.0) * vh_w
                    if trapezoid_orientation[-1] == 0:
                        v0 = [X[0] + a*V[0] + b*V[1], X[1] + a*V[1] - b*V[0]]
                        v1 = [X[0] - a*V[0] + b*V[1], X[1] - a*V[1] - b*V[0]]
                        v2 = [X[0] - a*V[0] - 3*vh_w*V[1], X[1] - a*V[1] + 3*vh_w*V[0]]
                        v3 = [X[0] + a*V[0] - 3*vh_w*V[1], X[1] + a*V[1] + 3*vh_w*V[0]]
                        v2 = [v2[0] - 3*vh_w*d*V[0], v2[1] - 3*vh_w*d*V[1]]   # at lane 0
                        v3 = [v3[0] + 3*vh_w*d*V[0], v3[1] + 3*vh_w*d*V[1]]
                    else:
                        v0 = [X[0] + a*V[0] + 3*vh_w*V[1], X[1] + a*V[1] - 3*vh_w*V[0]]
                        v1 = [X[0] - a*V[0] + 3*vh_w*V[1], X[1] - a*V[1] - 3*vh_w*V[0]]
                        v2 = [X[0] - a*V[0] - b*V[1], X[1] - a*V[1] + b*V[0]]
                        v3 = [X[0] + a*V[0] - b*V[1], X[1] + a*V[1] + b*V[0]]
                        v0 = [v0[0] + 3*vh_w*d*V[0], v0[1] + 3*vh_w*d*V[1]]   # at lane 1
                        v1 = [v1[0] - 3*vh_w*d*V[0], v1[1] - 3*vh_w*d*V[1]]
                    bar = [v0, v1, v2, v3, v0]
                    self.drawTrajectory(bar, lines, -0.8)
                '''

        newTraj = opt.CFS(pos, preTraj, obstacles, cq = [0.05,0,0], cs = [0.05,0.05,7], theta = 1.2,
        minimal_dis = 2.4, maxIter = 10, SCCFS = True, slack_w = 1.3, stop_eps = 0.1,
         trapezoid_orientation = trapezoid_orientation, xrec = self.xrec)
        
        # Draw CFS output trajectory
        if self._draw_traj:
            # self.drawTrajectory(preTraj, lines, -0.8)
            self.drawTrajectory(newTraj, lines, -0.8)
            trajNode = lines.create()    # panda3d draw
            self.trajNp = NodePath(trajNode)    # panda3d draw
            self.trajNp.reparentTo(render)    # panda3d draw
        if self._set_timer:
            print("plan time : ", time.perf_counter()-start_time)
        return newTraj

    def doControl(self):        
        return self.previewController()
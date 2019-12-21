################
# road.py
# this file contains classes to define and generate the road
#
# Author: Jianyu Chen
# Copyright: 2016
###############

from __future__ import division
from panda3d.core import *
import numpy as np
import math

# generate the boundary and texture of a road according to the centerline and lane width
class roadGenerator:
    def __init__(self,previousPoint,laneWidth,laneNum,followingPoint,segLine,height,texLength,precision):
        width=laneWidth*laneNum/2
        segLength=len(segLine)
        texPieceNum=int(math.floor(texLength/precision))
        texNum=int(segLength/texPieceNum)
        num=texPieceNum*texNum
        wholeSeg=[previousPoint]+segLine+[followingPoint]
        leftPoints=[]
        rightPoints=[]
        for i in range(1,segLength+1):
            direct1=wholeSeg[i-1]-wholeSeg[i]
            direct2=wholeSeg[i+1]-wholeSeg[i]
            direct1=direct1/math.sqrt(direct1[0]*direct1[0]+direct1[1]*direct1[1])
            direct2=direct2/math.sqrt(direct2[0]*direct2[0]+direct2[1]*direct2[1])

            if abs(direct1[0]+direct2[0])>0.01 and abs(direct1[1]+direct2[1])>0.01:
                if -direct1[0]*direct2[1]+direct2[0]*direct1[1]<0:
                    rightFlag=1
                else:
                    rightFlag=0
                pre=wholeSeg[i]+direct1
                po=wholeSeg[i]+direct2
                mi=(pre+po)/2
                direct=mi-wholeSeg[i]
                direct=direct/math.sqrt(direct[0]*direct[0]+direct[1]*direct[1])
            else:
                direct=np.array([direct2[1],-direct2[0]])
                direct=direct/math.sqrt(direct[0]*direct[0]+direct[1]*direct[1])
                rightFlag=1
            if rightFlag==1:
                rightPoints.append(wholeSeg[i]+direct*width)
                leftPoints.append(wholeSeg[i]-direct*width)
            else:
                rightPoints.append(wholeSeg[i]-direct*width)
                leftPoints.append(wholeSeg[i]+direct*width)
        

        self.node = GeomNode('gnode')


        for j in range(0,texNum):

            array = GeomVertexArrayFormat()
            array.addColumn("vertex", 3, Geom.NTFloat32, Geom.CPoint)
            array.addColumn("texcoord", 2, Geom.NTFloat32, Geom.CTexcoord)
            format = GeomVertexFormat()
            format.addArray(array)
            format = GeomVertexFormat.registerFormat(format)
            vdata = GeomVertexData('name', format, Geom.UHStatic)
            vdata.setNumRows(2*texPieceNum+2)
            vertex = GeomVertexWriter(vdata, 'vertex')
            texcoord = GeomVertexWriter(vdata, 'texcoord')
            for i in range(0,texPieceNum+1):
                vertex.addData3f(leftPoints[texPieceNum*j+i][0],leftPoints[texPieceNum*j+i][1],height)
                texcoord.addData2f(0, 1/(texPieceNum-1)*i)
                vertex.addData3f(rightPoints[texPieceNum*j+i][0],rightPoints[texPieceNum*j+i][1],height)
                texcoord.addData2f(1, 1/(texPieceNum-1)*i)
            prim = GeomTristrips(Geom.UHStatic)
            for i in range(0,2*texPieceNum+2):
                prim.addVertex(i)

            geom = Geom(vdata)
            geom.addPrimitive(prim)
            self.node.addGeom(geom)
        
        
        self.leftPoints=leftPoints
        self.rightPoints=rightPoints

    def getNode(self):
        return self.node

# Generate the centerline polyline of a curve road according to the radiu and angle
class curveCenter:
    def __init__(self,prevPoint,start,radiu,angle,pieceWidth,texLength,lf):
        rdirect=np.array([(start-prevPoint)[1],-(start-prevPoint)[0]])
        rdirect=rdirect/math.sqrt(rdirect[0]*rdirect[0]+rdirect[1]*rdirect[1])
        texNum=math.ceil(angle*radiu/texLength)
        texLength=angle*radiu/texNum
        unitAngle=angle/texNum
        pieceNum=math.floor(texLength/pieceWidth)
        pieceAngle=unitAngle/pieceNum
        self.segLine=[]
        if abs(-rdirect[0])<1e-5:
            if -rdirect[1]>0:
                initAngle=math.pi/2
            else:
                initAngle=-math.pi/2
        else:
            
            if -rdirect[1]>1e-5:
                initAngle=math.atan(rdirect[1]/rdirect[0])
            elif abs(-rdirect[1])<=1e-5:
                if -rdirect[0]>0:
                    initAngle=0
                else:
                    initAngle=math.pi
            else:
                initAngle=math.atan(rdirect[1]/rdirect[0])+math.pi
            
        if lf==1:
            circle=start+rdirect*radiu
            for i in range(1,int(pieceNum*texNum+1)):
                currentDirect=initAngle-i*pieceAngle
                currentDirect=np.array([math.cos(currentDirect),math.sin(currentDirect)])
                currentDirect=currentDirect/math.sqrt(currentDirect[0]*currentDirect[0]+currentDirect[1]*currentDirect[1])
                point=circle+currentDirect*radiu
                self.segLine.append(point)
        
        else:
            circle=start-rdirect*radiu
            for i in range(1,int(pieceNum*texNum+1)):
                currentDirect=initAngle+i*pieceAngle
                currentDirect=np.array([math.cos(currentDirect),math.sin(currentDirect)])
                currentDirect=currentDirect/math.sqrt(currentDirect[0]*currentDirect[0]+currentDirect[1]*currentDirect[1])
                point=circle+currentDirect*radiu
                self.segLine.append(point)                
                
    def getLine(self):
        return self.segLine

    def getStart(self):
        return self.segLine[0]

    def getEnd(self):
        return self.segLine[-1]

# Generate a straight centerline polyline
class straightCenter:
    def __init__(self,start,angle,length,precision):
        self.start=start
        self.end=start+np.array([math.cos(angle)*length,math.sin(angle)*length])
        #dotNum=int(math.floor(length/texlength))
        self.Line=[start]
        num=int(math.floor(length/precision))
        for i in range(1,num+1):
            self.Line.append((self.end-self.start)/num*i+self.start)
    def getStart(self):
        return self.start
    def getEnd(self):
        return self.Line[-1]
    def getLine(self):
        return self.Line
    def getFollowing(self):
        return self.getEnd()+(self.Line[-1]-self.Line[-2])

# Generate a round trail centerline
class trail:
    def __init__(self,length,width,radiu,precision):
        line1=straightCenter(np.array([0,0]),math.pi/2,length,precision)
        curve1=curveCenter(line1.getStart(),line1.getEnd(),radiu,math.pi/2,precision,2,1)
        line2=straightCenter(curve1.getEnd(),0,width,precision)
        curve2=curveCenter(line2.getStart(),line2.getEnd(),radiu,math.pi/2,precision,2,1)
        line3=straightCenter(curve2.getEnd(),-math.pi/2,length,precision)
        curve3=curveCenter(line3.getStart(),line3.getEnd(),radiu,math.pi/2,precision,2,1)
        line4=straightCenter(curve3.getEnd(),-math.pi,width,precision)
        curve4=curveCenter(line4.getStart(),line4.getEnd(),radiu,math.pi/2,precision,2,1)    
        self.segLine=[np.array([0,0])]+curve1.getLine()+curve2.getLine()+curve3.getLine()+curve4.getLine()

    def getLine(self):
        return self.segLine

# Generate a freeway with some straight road and curve road
class basicFreeWay:
    def __init__(self,length,radiu,repeat,precision,texlength):
        curve=[]
        line=[straightCenter(np.array([0,0]),math.pi/2,length,precision)]
        self.segLine=line[0].getLine()[0:-1]
        for i in range(0,repeat):
            curve.append(curveCenter(line[2*i].getStart(),line[2*i].getEnd(),radiu,math.pi/6,precision,texlength,1))
            line.append(straightCenter(curve[2*i].getEnd(),math.pi/3,length,precision))
            curve.append(curveCenter(line[2*i+1].getStart(),line[2*i+1].getEnd(),radiu,math.pi/6,precision,texlength,0))
            line.append(straightCenter(curve[2*i+1].getEnd(),math.pi/2,length,precision))
            self.segLine=self.segLine+curve[2*i].getLine()[0:-1]+line[2*i+1].getLine()[0:-1]+curve[2*i+1].getLine()[0:-1]+line[2*i+2].getLine()[0:-1]
        self.segLine=self.segLine+[line[-1].getEnd()]
    def getLine(self):
        return self.segLine
    def getFollowing(self):
        return self.segLine[-1]+2*(self.segLine[-1]-self.segLine[-2])
          
        
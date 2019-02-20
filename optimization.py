####################
# optimization.py
# this file implements the CFS algorithm
#
# Author: Changliu Liu
# Copyright: 2016
####################


import math
import numpy as np
import utility
from cvxopt import matrix, solvers
import matplotlib.pyplot as plt
solvers.options['show_progress'] = False

# First Order Model
def CFS_FirstOrder(x0, refTraj, obs = [], horizon = 25, ts = 1, maxIter = 10):
        
    A = matrix([1, 0, 0, 1],(2,2))
    B = matrix([ts, 0, 0, ts],(2,2))
    Q = matrix([2, 1, 1, 2],(2,2))
    nstate, nu = 2, 2
        
    refInput = [0]*horizon*nu #np.zeros((1,horizon*nu))
    for i in range(horizon):
        if i == 0:
            refInput[i*nu:(i+1)*nu] = utility.substruct(refTraj[i],x0)
        else:
            refInput[i*nu:(i+1)*nu] = utility.substruct(refTraj[i],refTraj[i-1])
    refInput = np.array(refInput)
    refInput = refInput/ts
    
    refTraj = np.reshape(np.array(refTraj), horizon*nu)

    # augmented system
    Aaug, Baug, Qaug = A, np.zeros((horizon*nstate,horizon*nu)), np.zeros((horizon*nstate,horizon*nstate))
    for i in range(horizon):
        if i>0:
            Aaug = np.vstack((Aaug,A**(i+1)))
        if i<horizon-1:
            Qaug = utility.embed(Qaug,np.array(Q),i*nstate,i*nstate)
        else:
            # Put large terminal cost
            Qaug = utility.embed(Qaug,100*np.array(Q),i*nstate,i*nstate)

        for j in range(i+1):
            
            newB = utility.matrixPower(A,i-j)*B
            Baug = utility.embed(Baug,np.array(newB),i*nstate,j*nu)
    
   
    # set up constraint
    P = matrix([25, 0, 0, 1],(2,2))
    p = np.dot(np.dot(Aaug,x0)-refTraj,np.dot(Qaug,Baug))

    Qopt = np.dot(np.transpose(Baug),np.dot(Qaug,Baug))+np.identity(nu*horizon)/10
    
    #print(np.dot(Aaug,x0)-refTraj+np.dot(Baug,refInput))
    #print(np.dot(refInput,np.dot(Qopt,refInput))+np.dot(p,refInput)*2+np.dot(np.dot(Aaug,x0)-refTraj,np.dot(Qaug,np.dot(Aaug,x0)-refTraj)))
    
    Qopt = matrix(Qopt,(nu*horizon,nu*horizon),'d')
    p = matrix(p,(nu*horizon,1),'d')

    
    iter = 0
    while iter < maxIter:
        iter += 1
        #print('Interation %d'%iter)
        Lstack, Sstack, D = [],[],100
        for j in range(len(obs)):
            for i in range(horizon):
                if i < horizon-1:
                    orient = utility.vec2ang(utility.substruct(obs[j][i+1],obs[j][i]))
                else:
                    orient = utility.vec2ang(utility.substruct(obs[j][i],obs[j][i-1]))
                R = np.matrix([[math.cos(orient), -math.sin(orient)],[math.sin(orient), math.cos(orient)]])
                
                NewP = matrix(np.dot(np.transpose(R),np.dot(P,R)),(2,2))
                #print(R,np.dot(P,R),NewP)
                
                I = np.dot(utility.matrixPower(A,i+1),x0)-obs[j][i]
                Bj = np.array(Baug)[i*nstate:(i+1)*nstate][0:horizon*nu]
                
                Bu = np.dot(Bj,refInput)
                
                s = -D+np.dot(I,np.dot(NewP,I))-np.dot(Bu,np.dot(NewP,Bu))
                #print(s)
                Sstack.append(s)
                l = -2*np.dot(I,np.dot(NewP,Bj))-2*np.dot(Bu,np.dot(NewP,Bj))
                Lstack.append(list(l))
                

        Lstack = matrix(Lstack,(len(Lstack[0]),len(Lstack)),'d')
        Lstack = Lstack.trans()
        Sstack = matrix(Sstack,(len(Sstack),1),'d')
        
    #print(np.amax(utility.substruct(np.dot(Lstack,refInput),np.array(Sstack))),np.argmax(utility.substruct(np.dot(Lstack,refInput),np.array(Sstack))))
        

        sol = solvers.qp(Qopt,p, Lstack,Sstack)
        newInput = sol['x']
        
        
        newInput = np.reshape(newInput,horizon*nu)
        if np.linalg.norm(newInput-refInput)<0.1:
            #print('converge at step %d'%iter)
            break
        refInput = newInput
    
    
    # Get new trajectory
    traj = np.zeros((horizon, nstate))
    for i in range(horizon):
        if i == 0:
            traj[i] = np.dot(A,x0)+np.dot(B,refInput[i*nu:(i+1)*nu])
        else:
            traj[i] = np.dot(A,traj[i-1])+np.dot(B,refInput[i*nu:(i+1)*nu])

    return traj

'''
#print(getTrajectory([0,0],[[1,0],[2,0]],[[[1,0.5]]*2],2))
traj = CFS_FirstOrder([0,0],[[1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0]],[[[2,0.5]]*10,[[4,-0.5]]*10],10)
traj = np.array(matrix(traj).trans())
print(traj[0],traj[1])
fig1 = plt.figure()
plt.plot(traj[0],traj[1],'r-')
#plt.xlim(0, 6)
#plt.ylim(-1, 1)
plt.xlabel('x')
plt.title('test')
plt.show()
'''

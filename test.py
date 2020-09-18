# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 09:41:31 2020

@author: zhy
"""
import numpy as np
from cvxopt import matrix, solvers
import DCFS as dcfs
import math
import utility
import matplotlib.pyplot as plt
import time



###1: Obstacle representation
#vh_l = 2.8 + 1.0
#vh_w = 1.2 + 0.6        
#        
#a = vh_l / 2
#b = vh_w / 2
#X = [0,0] 
#V = [0,1]
#
#if np.linalg.norm(V)==0:       
#    v0 = [X[0] - vh_w / 2, X[1] + vh_l / 2]
#    v1 = [X[0] + vh_w / 2, X[1] + vh_l / 2]
#    v2 = [X[0] + vh_w / 2, X[1] - vh_l / 2]
#    v3 = [X[0] - vh_w / 2, X[1] - vh_l / 2]    
#else:    
#    V = utility.normalize(V)
#    # vehicle polygon: rectangle
#    v0 = [X[0] + a*V[0] + b*V[1], X[1] + a*V[1] - b*V[0]+ vh_l / 2]   # upper right
#    v1 = [X[0] - a*V[0] + b*V[1], X[1] - a*V[1] - b*V[0]- vh_l / 2]   # lower right
#    v2 = [X[0] - a*V[0] - b*V[1], X[1] - a*V[1] + b*V[0]- vh_l / 2]   # lower left
#    v3 = [X[0] + a*V[0] - b*V[1], X[1] + a*V[1] + b*V[0]+ vh_l / 2]   # upper left  
#v = np.array([v0,v1,v2,v3])
#x = v[:,0]
#y = v[:,1]
#plt.plot(x,y,'b-*', label = "v")
#plt.axis('equal')
    
 
    

###2: D-CFS
multi_path = np.zeros((2, 20, 2)) 
share_path = np.zeros((2, 20, 2)) 
traj = np.zeros((2, 20, 2)) 
[num_veh, horizon, dim] = multi_path.shape

            
for j in range(horizon):
#    multi_path[0][j][1] = j*4
#    multi_path[1][j][1] = j*2+10
#    multi_path[2][j][1] = j*2+10
#    multi_path[2][j][0] = -5
#    share_path[0][j][1] = j*4
#    share_path[1][j][1] = j*2+10
#    share_path[2][j][1] = j*2+10
#    share_path[2][j][0] = -5
#    traj[0][j][1] = j*4
#    traj[1][j][1] = j*2+10
#    traj[2][j][1] = j*2+10
#    traj[2][j][0] = -5     
    multi_path[0][j][1] = j
    multi_path[1][j][0] = j-10
    multi_path[1][j][1] = 10

    share_path[0][j][1] = j
    share_path[1][j][0] = j-10
    share_path[1][j][1] = 10

    traj[0][j][1] = j*4
    traj[1][j][0] = j-10
    traj[1][j][1] = 10
 
dt = 0.02
ts = 0.02
start = math.ceil(dt/ts)    

modified_shared_path = np.zeros((num_veh, horizon, dim))

for i in range(num_veh):
    modified_shared_path[i][:horizon-start] = share_path[i][start:]
    modified_shared_path[i][horizon-start:] = share_path[i][-1]
veh_index = 0 
pos = modified_shared_path[veh_index][0]
x_ref = modified_shared_path[veh_index] 
#pos[0] += 0
#pos[1] -= 5

sol = dcfs.Opt_solver(pos, x_ref, veh_index, share_path, ts, dt)
traj[veh_index] = sol
   
traj0 = np.transpose(traj[0])
traj1 = np.transpose(modified_shared_path[1])
plt.plot(traj0[0],traj0[1],'b-*', label = "traj0")
plt.plot(traj1[0],traj1[1],'m-*', label = "traj1")
#plt.axis('equal')




###3
#modified_share_path = np.zeros((num_veh, horizon, dim))
#for i in range(num_veh):
#    modified_share_path[i][:horizon-start] = share_path[i][start:]


#precision = 1
#desireV = 20
#ts = 0.1
#h = 4
#num = 1
#space = math.ceil(ts*desireV/precision)
#ref_path = multi_path[0][num:(num+space*h):space]



###4: Centralized CFS
#SCCFS = True          
#veh_num = 2       
#horizon = 20
#dim = 2
#
#SV = veh_num*dim        
#
#ref_path = np.zeros((veh_num, horizon, dim)) 
#traj = np.zeros((veh_num, horizon, dim)) 
#
#            
#for j in range(horizon):
#    ref_path[0][j][1] = j*4
#    ref_path[1][j][1] = j*2+20
#    ref_path[0][j][0] = 0
##    ref_path[2][j][1] = j*2+20
#
##for j in range(horizon):
##    ref_path[0][j][1] = j*4
##    ref_path[1][j][1] = 40
##    ref_path[1][j][0] = j*4-40
#
#    
#pos = np.zeros(veh_num*dim)
#pos[0:2] = ref_path[0][0]
#pos[2:4] = ref_path[1][0]
##pos[4:6] = ref_path[2][0]
#
#x_ref = np.reshape(ref_path, (ref_path.size, 1))
#
#
#sol= dcfs.Centralized_solver(pos, ref_path, veh_num, horizon, dim, 0.02, 0, SCCFS = True)
#
##G = np.array(G)
##h = np.array(h)
##A = np.array(A)
##b = np.array(b)
#
#sol[1][8][1] = 38
#sol[1][9][1] = 39
#sol[1][10][0] = 4
#sol[1][11][1] = 40.5
#sol[1][12][1] = 41
#
#traj = np.reshape(sol, (veh_num, horizon, dim))
#   
#traj0 = np.transpose(traj[0])
#traj1 = np.transpose(traj[1])
##traj2 = np.transpose(traj[2])
#plt.plot(traj0[0],traj0[1],'b-*', label = "vehicle 0")
#plt.plot(traj1[0],traj1[1],'m-*', label = "vehicle 1")
#plt.plot(traj[0][0],traj[0][-1],'--', label = "Centerline")
#
#plt.xlabel('x')
#plt.ylabel('y')
#plt.legend(loc='upper right')
##plt.plot(traj2[0],traj2[1],'m-*', label = "traj2")
##plt.axis('equal')    
#plt.savefig('Overtaking',dpi = 400)





###5: Sve
#np.savez('traj_log.npz',traj = traj)
#data = np.load('traj_log.npz')
#print(data['traj'][0])

#from scipy import io
#mat = np.load('traj_log.npz')
#io.savemat('traj.mat', {'traj': traj})


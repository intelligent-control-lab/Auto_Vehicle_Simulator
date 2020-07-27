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



###1
#vh_l = 2.8 + 1.0
#vh_w = 1.2 + 0.6        
#        
#a = vh_l / 2
#b = vh_w / 2
#X = [0,0] 
#V = [0,0]
#
#if np.linalg.norm(V)==0:       
#    v0 = [X[0] - vh_w / 2, X[1] + vh_l / 2]
#    v1 = [X[0] + vh_w / 2, X[1] + vh_l / 2]
#    v2 = [X[0] + vh_w / 2, X[1] - vh_l / 2]
#    v3 = [X[0] - vh_w / 2, X[1] - vh_l / 2]    
#else:    
#    V = utility.normalize(V)
#    # vehicle polygon: rectangle
#    v0 = [X[0] + a*V[0] + b*V[1], X[1] + a*V[1] - b*V[0]]   # upper right
#    v1 = [X[0] - a*V[0] + b*V[1], X[1] - a*V[1] - b*V[0]]   # lower right
#    v2 = [X[0] - a*V[0] - b*V[1], X[1] - a*V[1] + b*V[0]]   # lower left
#    v3 = [X[0] + a*V[0] - b*V[1], X[1] + a*V[1] + b*V[0]]   # upper left  
#v = np.array([v0,v1,v2,v3])
#x = v[:,0]
#y = v[:,1]
#plt.plot(x,y,'b-*', label = "v")
#plt.axis('equal')
    
    
    

###2
multi_path = np.zeros((3, 10, 2)) 
share_path = np.zeros((3, 10, 2)) 
traj = np.zeros((3, 10, 2)) 
[num_veh, horizon, dim] = multi_path.shape

            
for j in range(horizon):
    multi_path[0][j][1] = j*4
    multi_path[1][j][1] = j*2+10
    multi_path[2][j][1] = j*2+10
    multi_path[2][j][0] = -5
    share_path[0][j][1] = j*4
    share_path[1][j][1] = j*2+10
    share_path[2][j][1] = j*2+10
    share_path[2][j][0] = -5
    traj[0][j][1] = j*4
    traj[1][j][1] = j*2+10
    traj[2][j][1] = j*2+10
    traj[2][j][0] = -5     
    
dt = 0.3
ts = 0.1
start = math.ceil(dt/ts)    

modified_shared_path = np.zeros((num_veh, horizon, dim))

for i in range(num_veh):
    modified_shared_path[i][:horizon-start] = share_path[i][start:]
    modified_shared_path[i][horizon-start:] = share_path[i][-1]
veh_index = 0 
pos = modified_shared_path[veh_index][0]
x_ref = modified_shared_path[veh_index] 
pos[0] += 0
pos[1] -= 3

sol = dcfs.Opt_solver(pos, x_ref, veh_index, share_path, ts, dt)
traj[veh_index] = sol
#for veh_index in range(num_veh):
#    traj[veh_index] = share_path[veh_index]
#print(traj)    
traj0 = np.transpose(traj[0])
traj1 = np.transpose(modified_shared_path[1])
traj2 = np.transpose(modified_shared_path[2])
plt.plot(traj0[0],traj0[1],'b-*', label = "traj0")
plt.plot(traj1[0],traj1[1],'m-*', label = "traj1")
plt.plot(traj2[0],traj2[1],'m-*', label = "traj2")
#plt.axis('equal')



###3
#for veh_index in range(num_veh):
#    
#    pos = share_path[veh_index][0]    
#    x_ref = multi_path[veh_index]
#
##    P, q = dcfs.Opt_obj_func(x_ref, cq = [1,0,1], cs = [1,0,1], ts = 1, SCCFS = False, slack_w = 1.0)
##    G, h, A, b = dcfs.Opt_constraints(pos, veh_index, start, traj, ts = 1, min_dis = 20) 
#    #print(P)
#    #print(q)
#    #print(G)
#    #print(h)
#    #print(A)
#    #print(b)
#    x_ts = dcfs.Opt_solver(pos, x_ref, veh_index, traj, ts, 0.4)
#    
##    sol = solvers.qp(P, q, G, h, A, b)
##    x_ts = sol['x']
##    x_ts = np.reshape(x_ts, (horizon,dim))
#    share_path[veh_index] = x_ts
##    print(traj)

#for veh_index in range(num_veh):
#    traj[veh_index] = share_path[veh_index]
##print(traj)    
#traj0 = np.transpose(traj[0])
#traj1 = np.transpose(traj[1])
#plt.plot(traj0[0],traj0[1],'b-*', label = "traj0")
#plt.plot(traj1[0],traj1[1],'m-*', label = "traj1")
##plt.axis('equal')


###4
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

      
    

#ego_pos = [0,0]
#obs_pos = [10,10]
#
#obs_vel = [1,0]
#line_set = dcfs.convex_hull_2d_2_feasible_set(ego_pos, obs_pos, obs_vel)
#        
#x = line_set[0][0][0]                
#y = line_set[0][0][1]              
#const = line_set[0][1]


          
            

            
            
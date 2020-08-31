# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 13:45:02 2020

CFS_DMPC 
Simulation for planning
Assuming perfect control is available

@author: Hongyu Zhou
"""

from scipy import io
import numpy as np
import DCFS as dcfs
import math
import matplotlib.pyplot as plt
import time



# Vehicle parameters
horizon = 20
dim = 2
ts = 0.05
dt = ts
desiredV = 10


# 0: Intersection; 2 T_r to reach consensus
# 1: Unstructed Road; 3 T_r to reach consensus
# 2: Overtaking
Lane_switch = 2

# Lane
lane_num = 4
lane_width = 4
lane_length = 50

space = ts*desiredV
point_num = math.floor(lane_length/space)+1

lane_dir = np.array([[0,1] , [0,-1] , [1,0] , [-1,0]])
    
if Lane_switch is 0: 
    lane_init = np.array([[lane_width/2,0] , [-lane_width/2,lane_length] , 
                 [-lane_length/2,lane_length/2-lane_width/2] , [lane_length/2,lane_length/2+lane_width/2]])
    target_lane = [0 , 1 , 2 , 3]


elif Lane_switch is 1:
    lane_init = np.array([[0,-lane_length/2] , [0,lane_length/2] , 
                 [-lane_length/2,0] , [lane_length/2,0]])
    target_lane = [0 , 2]
    point_num += 10
    
elif Lane_switch is 2:
    lane_num = 5
    desiredV = [20,10,10,10,10]
    lane_dir = np.array([[0,1] , [0,1] , [0,1] , [0,1] , [0,1]])
    lane_init = np.array([[0,0] , [0,10] , [-4,20] , [4,30] , [0,50]])
    target_lane = [0 , 1 , 2 , 3, 4]
    space = np.array(desiredV)*ts
    point_num += 50

    
lane = np.zeros([lane_num,point_num,dim])
for i in range(lane_num):
    for j in range(point_num):
        lane[i][j] = lane_init[i] + lane_dir[i] * j * space[i]
        
#lane0 = np.transpose(lane[0])
#lane1 = np.transpose(lane[1])
#lane2 = np.transpose(lane[2])
#lane3 = np.transpose(lane[3])
#plt.plot(lane0[0],lane0[1],'b-')
#plt.plot(lane1[0],lane1[1],'m-')    
#plt.plot(lane2[0],lane2[1],'k-')
#plt.plot(lane3[0],lane3[1],'r-')    
      
 
    
# Planning
traj_log = []
num_veh = len(target_lane)
shared_path = np.zeros([num_veh, horizon, dim])
ref_path = np.zeros([num_veh, horizon, dim])
traj_sol = np.zeros([num_veh, horizon, dim])

k = 0 # index for replanning
k_max = point_num-horizon-10

for i in range(num_veh):
    ref_path[i] = lane[target_lane[i]][k:k+horizon]
    shared_path[i] = lane[target_lane[i]][k:k+horizon]   
    traj_sol[i] = lane[target_lane[i]][k:k+horizon]   
    
# Communication    
def communication(traj_sol):    
    return traj_sol

# Planning loop
while k<=k_max:
    
    # Share traj
    shared_path = communication(traj_sol)
    
    # Planning
    for veh_index in range(num_veh):
        if k == 0:
            pos = traj_sol[veh_index][0]
        else:
            pos = traj_sol[veh_index][1]
        traj_log = np.append(traj_log,(pos))
#        print(veh_index,pos)
        
        traj_sol[veh_index] = dcfs.Opt_solver(pos, ref_path[veh_index], veh_index, shared_path, ts, dt)
    
    # Plot
    traj_log0 = np.transpose(traj_sol[0])
    traj_log1 = np.transpose(traj_sol[1])
    plt.plot(traj_log0[0],traj_log0[1],'b-')
    plt.plot(traj_log1[0],traj_log1[1],'m-') 
    if Lane_switch is 0: 
        traj_log2 = np.transpose(traj_sol[2])
        traj_log3 = np.transpose(traj_sol[3])
        plt.plot(traj_log2[0],traj_log2[1],'g-')
        plt.plot(traj_log3[0],traj_log3[1],'k-') 
    if Lane_switch is 2: 
        traj_log2 = np.transpose(traj_sol[2])
        traj_log3 = np.transpose(traj_sol[3])
        traj_log4 = np.transpose(traj_sol[4])
        plt.plot(traj_log2[0],traj_log2[1],'g-')
        plt.plot(traj_log3[0],traj_log3[1],'k-')           
        plt.plot(traj_log4[0],traj_log4[1],'k-')           
    plt.pause(0.05)
    
#    if k == 28:
#        io.savemat('CFS_DMPC_Intersection_28.mat', {'traj_sol': traj_sol})
#    if k == 30:
#        io.savemat('CFS_DMPC_Intersection_30.mat', {'traj_sol': traj_sol})
        
#    if k == 25:
#        io.savemat('CFS_DMPC_UnstructedRoad_25.mat', {'traj_sol': traj_sol})
#    if k == 28:
#        io.savemat('CFS_DMPC_UnstructedRoad_28.mat', {'traj_sol': traj_sol})
        
        
    k = k+1
    print('k',k)
    if k == k_max:
        break;
    
    # Shift ref path    
    for i in range(num_veh):
        ref_path[i] = lane[target_lane[i]][k:k+horizon]



# Save        
length = len(traj_log)
num_time_step = int(length/(dim*num_veh))
traj_log = np.reshape(traj_log,(num_time_step,dim*num_veh))      

io.savemat('CFS_DMPC_Traj.mat', {'traj_log': traj_log})
    
# Plot
traj_log0 = np.transpose(traj_log)[0:2]
traj_log1 = np.transpose(traj_log)[2:4]
plt.plot(traj_log0[0],traj_log0[1],'b-')
plt.plot(traj_log1[0],traj_log1[1],'m-') 
if Lane_switch is 0: 
    traj_log2 = np.transpose(traj_log)[4:6]
    traj_log3 = np.transpose(traj_log)[6:8]
    plt.plot(traj_log2[0],traj_log2[1],'g-')
    plt.plot(traj_log3[0],traj_log3[1],'k-')  
if Lane_switch is 2: 
    traj_log2 = np.transpose(traj_sol[2])
    traj_log3 = np.transpose(traj_sol[3])
    traj_log4 = np.transpose(traj_sol[4])
    plt.plot(traj_log2[0],traj_log2[1],'g-')
    plt.plot(traj_log3[0],traj_log3[1],'k-')           
    plt.plot(traj_log4[0],traj_log4[1],'y-') 
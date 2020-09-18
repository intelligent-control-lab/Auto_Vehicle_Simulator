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
import math
import time
import matplotlib.pyplot as plt
import CFS_DMPC_Solver 



# Parameters
horizon = 10
dim = 2
ts = 0.1
dt = ts


# 1: Unstructed Road
# 2: Overtaking
# 3: Platoon formation
Lane_switch = 1

# Lane

#elif Lane_switch is 1:
#    lane_num = 3
#    lane_length = 30
#    desiredV = [10,10,10]
#    space = np.array(desiredV)*ts
#    point_num = 100     
#    lane_dir = np.array([[0,1] , [-1/math.sqrt(2),-1/math.sqrt(2)] , [1,0]])
#    lane_init = np.array([[0,-lane_length/2] , [lane_length/(2*math.sqrt(2)),lane_length/(2*math.sqrt(2))] , 
#                 [-lane_length/2,0]])
#    target_lane = [0 , 1 , 2]   

if Lane_switch is 1:
    lane_num = 3
    r = 20
    desiredV = [10]*lane_num
    space = np.array(desiredV)*ts
    point_num = 200
    a = 2
    lane_dir = np.array([[math.cos(i*a*math.pi/lane_num+math.pi),math.sin(i*a*math.pi/lane_num+math.pi)] for i in range(lane_num)])
    lane_init = np.array([[r*math.cos(i*a*math.pi/lane_num),r*math.sin(i*a*math.pi/lane_num)] for i in range(lane_num)])
    target_lane = np.array([i for i in range(lane_num)])  
    

elif Lane_switch is 2:
    horizon = 20
    lane_num = 5
    desiredV = [20,10,10,10,10]
    space = np.array(desiredV)*ts
    point_num = 100 
    lane_dir = np.array([[0,1] , [0,1] , [0,1] , [0,1] , [0,1]])
    lane_init = np.array([[0,0] , [0,10] , [-4,20] , [4,30] , [0,50]])
    target_lane = [0 , 1 , 2 , 3]
   

elif Lane_switch is 3:
    lane_num = 5
    desiredV = [20,20,20,20,20]
    space = np.array(desiredV)*ts
    point_num = 100
    lane_dir = np.array([[0,1] , [0,1] , [0,1] , [0,1] , [0,1]])
    lane_init = np.array([[0,0] , [0,6] , [0,12] , [0,18] , [0,24]])
    target_lane = [0 , 1 , 2 , 3 , 4]
    

    
lane = np.zeros([lane_num,point_num,dim])
for i in range(lane_num):
    for j in range(point_num):
        lane[i][j] = lane_init[i] + lane_dir[i] * j * space[i]
           
      
 
    
# Planning
traj_log = []
num_veh = len(target_lane)
shared_path = np.zeros([num_veh, horizon, dim])
ref_path = np.zeros([num_veh, horizon, dim])
traj_sol = np.zeros([num_veh, horizon, dim])

k = 0 # index for replanning
k_max = 61

for i in range(num_veh):
    ref_path[i] = lane[target_lane[i]][k:k+horizon]
    shared_path[i] = lane[target_lane[i]][k:k+horizon]   
    traj_sol[i] = lane[target_lane[i]][k:k+horizon]   
    
# Communication    
def communication(traj_sol):    
    return traj_sol

# Planning loop
while k <= k_max:
    
    # Share traj
    shared_path = communication(traj_sol)
    
    # Planning
    for veh_index in range(num_veh):
        if k == 0:
            if Lane_switch is 3:
                pos = traj_sol[veh_index][0]+(-1)**veh_index*np.array([4,0])
            else:
                pos = traj_sol[veh_index][0]                    
        else:
            pos = traj_sol[veh_index][1]                                 
                    
            
        traj_log = np.append(traj_log,(pos))

        start = time.perf_counter()
                
        traj_sol[veh_index] = CFS_DMPC_Solver.Opt_solver(pos, ref_path[veh_index], veh_index, shared_path, ts, dt, True)        

        end = time.perf_counter()
#        print(end-start)


                
    # Plot
    traj_log0 = np.transpose(traj_sol[0])
    traj_log1 = np.transpose(traj_sol[1])
    traj_log2 = np.transpose(traj_sol[2])
    plt.plot(traj_log0[0],traj_log0[1],'b-',traj_log0[0][0],traj_log0[1][0],'*')
    plt.plot(traj_log1[0],traj_log1[1],'m-',traj_log1[0][0],traj_log1[1][0],'*') 
    plt.plot(traj_log2[0],traj_log2[1],'g-',traj_log2[0][0],traj_log2[1][0],'*')

    if Lane_switch is 2: 
        traj_log3 = np.transpose(traj_sol[3])
        plt.plot(traj_log3[0],traj_log3[1],'k-',traj_log3[0][0],traj_log3[1][0],'*') 

    elif Lane_switch is 3: 
        traj_log3 = np.transpose(traj_sol[3])
        plt.plot(traj_log3[0],traj_log3[1],'k-',traj_log3[0][0],traj_log3[1][0],'*') 
        traj_log4 = np.transpose(traj_sol[4])         
        plt.plot(traj_log4[0],traj_log4[1],'k-',traj_log4[0][0],traj_log4[1][0],'*')   
        
    plt.pause(0.05)
    
    
        
    print(k)    
    k = k+1
    if k == k_max:
        break;
        
        
        
    # Shift ref path    
    for veh_index in range(num_veh):
        if k < 1:
            start = math.floor(np.linalg.norm(traj_sol[veh_index][0]-lane[target_lane[veh_index]][0])/space[veh_index])
        else:
            start = math.floor(np.linalg.norm(traj_sol[veh_index][1]-lane[target_lane[veh_index]][0])/space[veh_index])
            
        ref_path[veh_index] = lane[target_lane[veh_index]][start:start+horizon]



# Save        
length = len(traj_log)
num_time_step = int(length/(dim*num_veh))
traj_log = np.reshape(traj_log,(num_time_step,dim*num_veh))      
io.savemat('CFS_DMPC_Traj.mat', {'traj_log': traj_log})
    

# Plot
traj_log0 = np.transpose(traj_sol[0])
traj_log1 = np.transpose(traj_sol[1])
traj_log2 = np.transpose(traj_sol[2])
plt.plot(traj_log0[0],traj_log0[1],'b-',traj_log0[0][0],traj_log0[1][0],'*')
plt.plot(traj_log1[0],traj_log1[1],'m-',traj_log1[0][0],traj_log1[1][0],'*') 
plt.plot(traj_log2[0],traj_log2[1],'g-',traj_log2[0][0],traj_log2[1][0],'*')

if Lane_switch is 2: 
    traj_log3 = np.transpose(traj_sol[3])
    plt.plot(traj_log3[0],traj_log3[1],'k-',traj_log3[0][0],traj_log3[1][0],'*') 
elif Lane_switch is 3: 
    traj_log3 = np.transpose(traj_sol[3])
    plt.plot(traj_log3[0],traj_log3[1],'k-',traj_log3[0][0],traj_log3[1][0],'*') 
    traj_log4 = np.transpose(traj_sol[4])         
    plt.plot(traj_log4[0],traj_log4[1],'k-',traj_log4[0][0],traj_log4[1][0],'*')   
    
    
    
    
    
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 19 10:52:45 2020

@author: Hongyu Zhou
"""

import time
import math
import numpy as np
import utility
from cvxopt import matrix, solvers
from itertools import combinations, permutations
solvers.options['show_progress'] = False


# Distributed MPC
def Opt_solver(pos, x_ref, veh_index, shared_path, ts, dt, SCCFS = True):
    '''
    Generate objective function in the form of 1/2x'Px+q'x. 
    Inputs:
        pos: vehicle position
        x_ref: the reference trajectory chosen as the centerline of the target lane
        veh_index: the vehicle index
        shared_path: the paths of all vehicles
        ts: MPC time step
        dt: simulation time step
    Outputs:
        x_sol: optimal traj
    '''
    
#    x_ref = shared_path[veh_index]
#    pos = shared_path[veh_index][0]
    [num_veh, horizon, dim] = shared_path.shape
    
    # shared_path should be shifted due to: ts != dt
    shifted_shared_path = np.zeros((num_veh, horizon, dim))
    start = math.ceil(dt/ts)
    for i in range(num_veh):
        shifted_shared_path[i][:horizon-start] = shared_path[i][start:]
        shifted_shared_path[i][horizon-start:] = shared_path[i][-1]        
#    print('shifted_shared_path:',shifted_shared_path) 
    
    # Optimization problem formulation

# DMPC vs MCCFS
#    c = np.array([1,0,100])/400000
#    P, q = Opt_obj_func(x_ref, cq = c, cs = c, ts = 1, SCCFS = SCCFS)

# DMPC vs RVO
#    [1,0,1000] :  8: 1
#    P, q = Opt_obj_func(x_ref, cq = [1,0,1000], cs = [1,0,1000], ts = 1, SCCFS = SCCFS)
#    [1 0 50] : 0.01-0.23  0.005  0.015  //  2 : 0.23  3 : <0.5   4 : 0.2  6: 0.33
#    P, q = Opt_obj_func(x_ref, cq = [1,0,50], cs = [1,0,50], ts = 0.5, SCCFS = SCCFS)

# Panda3d Sim    
    P, q = Opt_obj_func(x_ref, cq = [1,0,10], cs = [1,0,1], ts = 1, SCCFS = SCCFS)

    G, h, A, b = Opt_constraints(pos, veh_index, start, shifted_shared_path, ts, min_dis = 3, SCCFS = SCCFS) 

#    sol = solvers.qp(P, q, G, h)
    sol = solvers.qp(P, q, G, h, A, b)
    x_sol = sol['x']
    if SCCFS is True: # slack variables
        x_sol = sol['x'][:-2]
    x_sol = np.reshape(x_sol, (horizon, dim))
    
    primal = sol['primal objective']
#    print(primal,',...')
    
    return x_sol


    
def Opt_obj_func(x_ref, cq = [1,0,1], cs = [1,0,1], ts = 1, SCCFS = False, slack_w = 1.0):
    '''
    Generate objective function in the form of 1/2x'Px+q'x. 
    Inputs:
        x_ref: the reference trajectory
        cq, cs: tuning parameters
    Outputs:
        Matrix: P, q
    '''
    
    x_ref = np.array(x_ref)
    horizon = x_ref.shape[0]    
    dimension = x_ref.shape[1] # would be 2 (x and y)
    SV = 0 # slack variables
    x_ref = np.reshape(x_ref, (x_ref.size, 1)) # flatten to one dimension for applying qp, in the form of x0,y0,x1,y1,...
    if SCCFS is True: # slack variables
        x_ref = np.append(x_ref, [0]*2)
        SV = 2

    
    I = np.identity(horizon * dimension + SV)
    if SCCFS is True: # slack variables
        I[-2][-2] = 50
        I[-1][-1] = 50
    #    print(I)

    # Velocity is used to calculate the velocities at each time step
    Velocity = np.zeros(((horizon - 1) * dimension, horizon * dimension + SV))
    for i in range(len(Velocity)):
        Velocity[i][i] = 1.0
        Velocity[i][i + dimension] = -1.0
    Velocity /= ts
#    print(Velocity)
    
    # Acceleration is used to calculate the accelerations at each time step. See Eq(12) in the FOAD paper
    Acceleration = np.zeros(((horizon - 2) * dimension, horizon * dimension + SV))
    for i in range(len(Acceleration)):
        Acceleration[i][i] = 1.0
        Acceleration[i][i + dimension] = -2.0
        Acceleration[i][i + dimension + dimension] = 1.0
    Acceleration /= (ts * ts)
#    print(Acceleration)

    Q = cq[0] * I + cq[1] * np.dot(np.transpose(Velocity), Velocity) + cq[2] * np.dot(np.transpose(Acceleration), Acceleration)
    S = cs[0] * I + cs[1] * np.dot(np.transpose(Velocity), Velocity) + cs[2] * np.dot(np.transpose(Acceleration), Acceleration)


    # weight
    w1 = 1
    w2 = 1
    w3 = 50
    # Objective function
    P = w1 * Q + w2 * S # + w3 * C
    q = -2 * w1 * np.dot(Q, x_ref) # + w3 * Cf
#    print(P.shape)
#    print(q.shape)
#    print('P:',P)
#    print('q:',q)    
    
    P = matrix(P,(len(P),len(P[0])),'d')
    q = matrix(q,(len(q), 1),'d')

    return P, q


def Opt_constraints(pos, veh_index, start, shared_path, ts = 1, min_dis = 0.2, SCCFS = True):
    '''
    Generate constraints in the form of Gx <= h & Ax = b. 
    Inputs:
        pos: vehicle position
        veh_index: the vehicle index
        start: the number of MPC time step that should be skipped
        shared_path: the shifted paths of all vehicles
        min_dis: minimal distance
        ts: time step
    Outputs:
        Matrix: G, h, A, b
    '''
    [num_veh, horizon, dim] = shared_path.shape
    num_obs = num_veh-1
    
#    L = list(range(num_veh))
#    L.pop(veh_index)
#    obs_path = shared_path[L] 
    obs_path = np.delete(shared_path, veh_index, 0)
    ego_path = shared_path[veh_index]

    SV = 0 # slack variables
    if SCCFS is True: # slack variables
        SV = 2    
    
    # Inequality constraints: convex feasible set
    G = np.zeros((horizon * num_obs, horizon * dim + SV))
    h = np.zeros((horizon * num_obs, 1))
    
    start = 0
    for i in range(horizon-start):   # At each time step
        ego_pos = ego_path[i]
        
        for obs_index in range(num_obs):  # Consider all other vehicles            
            obs_pos= obs_path[obs_index][i]
            
            # the number of velocity vector is one less than the number of position vector
            if i == horizon-start-1:
                obs_pos_pre = obs_path[obs_index][i-1]
                obs_vel = (obs_pos - obs_pos_pre)/ts
            else:
                obs_pos_next = obs_path[obs_index][i+1]
                obs_vel = (obs_pos_next - obs_pos)/ts   

            line_set = convex_hull_2d_2_feasible_set(ego_pos, obs_pos, obs_vel)
            # line normal vector x, y                
            x = line_set[0][0][0]                
            y = line_set[0][0][1]                
            const = line_set[0][1]
#            print('normal vector:',x,y)
#            print('const:',const)

            G[i * num_obs + obs_index][i * dim] = -x
            G[i * num_obs + obs_index][i * dim + 1] = -y
            h[i * num_obs + obs_index] = -const-min_dis
            
#    print('G:',G)
#    print('h:',h)                
    G = matrix(G,(len(G),len(G[0])),'d')
    h = matrix(h,(len(h),1),'d')
    
    # Equality constraints: fix vehicle's initial position
    A = np.zeros((dim, horizon * dim + SV))
    b = np.zeros((len(A), 1))
           
    A[0][0] = 1
    A[1][1] = 1    
    if SCCFS is True: # slack variables
        A[0][-2] = 1
        A[1][-1] = 1
    b[0] = pos[0]
    b[1] = pos[1]
#    print(A.shape)
#    print(b.shape) 
#    print('A:',A)
#    print('b:',b)         
    A = matrix(A,(len(A),len(A[0])),'d')
    b = matrix(b,(len(b),1),'d')
    
    return G, h, A, b


# Obstacle representation
def convex_hull_2d_2_feasible_set(ego_pos, obs_pos, obs_vel = [0, 0]):
    '''
    Inputs:
        ego_pos: the position of ego vehicle 
        obs_pos: the position of other vehicles at the same time step
        obs_vel: the velocity of other vehicles
    Outputs:
        line_set: parameters for CFS, see function distancePointMesh
    '''
    
    line_set = []
    
    # vehicle parameter
    vh_l = 2.8 + 1.0  # 2.8 + 4.0
    vh_w = 1.2 + 0.8        
            
    a = vh_l / 2
    b = vh_w / 2
    X = obs_pos 
    
    if np.linalg.norm(obs_vel)==0:
        # vehicle polygon: rectangle
        v0 = [X[0] - vh_w / 2, X[1] + vh_l / 2]
        v1 = [X[0] + vh_w / 2, X[1] + vh_l / 2]
        v2 = [X[0] + vh_w / 2, X[1] - vh_l / 2]
        v3 = [X[0] - vh_w / 2, X[1] - vh_l / 2]    
    else:    
        V = utility.normalize(obs_vel)
        # vehicle polygon: rectangle align with velocity
        v0 = [X[0] + a*V[0] + b*V[1], X[1] + a*V[1] - b*V[0]]   # upper right
        v1 = [X[0] - a*V[0] + b*V[1], X[1] - a*V[1] - b*V[0]]   # lower right
        v2 = [X[0] - a*V[0] - b*V[1], X[1] - a*V[1] + b*V[0]]   # lower left
        v3 = [X[0] + a*V[0] - b*V[1], X[1] + a*V[1] + b*V[0]]   # upper left        

        
    v = [v0, v1, v2, v3]    # 4 vertices represent a surrounding vehicle
    normal, const, dist, land_point = distancePointMesh(ego_pos, v)
#    print('v:',v)
#    print('land_point:',land_point)
    line_set.append([normal, const])
    return line_set


def distancePointMesh(point, vertices):
# Input:
# point: a 2D point [x,y]
# vertices: an array of 2D points to represent a polygon
# 
# Output:
# ret_normal: a normalized normal vector points outward from the polygon
# ret_const: ret_normal * ret_land_point = ret_const
# ret_dist: min distance from the point to the line
# ret_land_point: a point on the line

    ret_dist = float('inf')
    ret_normal = []
    ret_land_point = []
    ret_const = 0
    ret_p = 0

    n_edge = len(vertices)
    for i in range(n_edge):
        x1 = vertices[i][0]
        y1 = vertices[i][1]
        x2 = vertices[(i + 1) % n_edge][0]
        y2 = vertices[(i + 1) % n_edge][1]

        trid = []
        trid += [np.linalg.norm([x1 - x2, y1 - y2])]
        trid += [np.linalg.norm([x1 - point[0], y1 - point[1]])]
        trid += [np.linalg.norm([x2 - point[0], y2 - point[1]])]

        # if the point is in between the line segment
        normal = [y1 - y2, x2 - x1]
        land_point = [x1, y1]
        const = -x1 * y2 + x2 * y1 # simply from normal * land_point'
        dist = abs(normal[0] * point[0] + normal[1] * point[1] - const) / trid[0]

        if trid[1]** 2 > trid[0]** 2 + trid[2]**2:
            dist = trid[2]
            normal = [point[0] - x2, point[1] - y2]
            const = np.dot(normal, np.transpose([x2, y2]))
            land_point = [x2, y2]

        if trid[2]** 2 > trid[0]**2 + trid[1]**2:
            dist = trid[1]
            normal = [point[0] - x1, point[1] - y1]
            const = np.dot(normal, np.transpose([x1, y1]))
        
        if dist < ret_dist:
            ret_dist = dist
            ret_normal = np.array(normal)
            ret_const = const
            ret_land_point = land_point
            ret_p = i

    # direction of normal vector, use a diagonal point to determine
    if np.dot(ret_normal, np.transpose(vertices[(ret_p + 2) % n_edge])) > ret_const:
        ret_normal = -ret_normal
        ret_const = -ret_const

    # normalization
    n = np.linalg.norm(ret_normal)
    ret_normal /= n
    ret_const /= n

    return ret_normal, ret_const, ret_dist, ret_land_point




## Centralized MPC
#def Centralized_solver(pos, x_ref, veh_num, horizon, dim, ts = 0.02, dt = 0.02, SCCFS = True):
#    '''
#    Generate objective function in the form of 1/2x'Px+q'x. 
#    Inputs:
#        pos: vehicle position with shape [veh_num , dim]
#        x_ref: the reference trajectory chosen as the centerline of the target lane with shape [veh_num , horizon, dim]
#        veh_num: the number of vehicles
#        horizon: the planning horizon
#        dim: 2
#        ts: MPC time step
#        dt: simulation time step
#    Outputs:
#        x_sol: optimal traj
#    '''
#    
#    start = time.perf_counter()
#
#    cq = [1,0,2]
#    cs = [1,0,1]
#    min_dis = 4
#    
##    # x_ref should be shifted 
##    start = 1
##    for i in range(veh_num):
##        x_ref[i][:horizon-start] = x_ref[i][start:]
##        x_ref[i][horizon-start:] = x_ref[i][-1]      
#    
#    SV = 0 # slack variables
#    pos = np.reshape(pos, (pos.size, 1))
#    x_ref = np.reshape(x_ref, (x_ref.size, 1)) # flatten to one dimension for applying qp, in the form of x0,y0,x1,y1,...
#    if SCCFS is True: # slack variables
#        x_ref = np.append(x_ref, [0] * veh_num * dim)
#        SV = veh_num * dim
##    print(x_ref)
#
#    # Objective function formulation
#    I = np.identity(x_ref.size)
#    if SCCFS is True: # slack variables
#        for i in range(SV):
#            I[veh_num * horizon * dim + i][veh_num * horizon * dim + i] = 10
##    print(I)
#
#    
#    # Acceleration is used to calculate the accelerations at each time step. See Eq(12) in the FOAD paper
#    Acceleration = np.zeros((veh_num * (horizon - 2) * dim, veh_num * horizon * dim + SV))
#    for i in range(veh_num):
#        for j in range((horizon - 2) * dim):
#            Acceleration[i * (horizon - 2) * dim + j][j + i * horizon * dim] = 1.0
#            Acceleration[i * (horizon - 2) * dim + j][j + i * horizon * dim + dim] = -2.0
#            Acceleration[i * (horizon - 2) * dim + j][j + i * horizon * dim + 2*dim] = 1.0
##    print(Acceleration)
#
#    Q = cq[0] * I + cq[2] * np.dot(np.transpose(Acceleration), Acceleration)
#    S = cs[0] * I + cs[2] * np.dot(np.transpose(Acceleration), Acceleration)
#
#
#    # weight
#    w1 = 1
#    w2 = 1
#
#    # Objective function
#    P = w1 * Q + w2 * S 
#    q = -2 * w1 * np.dot(Q, x_ref) 
##    print(P.shape)
##    print(q.shape)
##    print('P:',P)
##    print('q:',q)    
#    
#    P = matrix(P,(len(P),len(P[0])),'d')
#    q = matrix(q,(len(q), 1),'d')
#
#    end = time.perf_counter()
#    print('Set up:',end-start)
#    
#    # Constraints formulation
#    veh_pair = list(permutations(range(veh_num), 2))
#    veh_pair_num = len(veh_pair)
#    
#    # Inequality constraints: convex feasible set
#    G = np.zeros((horizon * veh_pair_num, veh_num * horizon * dim + SV))
#    h = np.zeros((horizon * veh_pair_num, 1))
#    
#    for i in range(veh_pair_num):   # At each time step
#        ego_index = veh_pair[i][0]
#        obs_index = veh_pair[i][1]        
#        for j in range(horizon):
#            ego_pos = x_ref[ego_index*horizon*dim + j*dim : ego_index*horizon*dim + j*dim+dim]
#            obs_pos = x_ref[obs_index*horizon*dim + j*dim : obs_index*horizon*dim + j*dim+dim]
#
#            line_set = convex_hull_2d_2_feasible_set(ego_pos, obs_pos, obs_vel = [0,0])
#            # line normal vector x, y                
#            x = line_set[0][0][0]                
#            y = line_set[0][0][1]                
#            const = line_set[0][1]
##            print('normal vector:',x,y)
##            print('const:',const)
#
#            G[i * horizon + j][ego_index*horizon*dim + j * dim] = -x
#            G[i * horizon + j][ego_index*horizon*dim + j * dim + 1] = -y
#            h[i * horizon + j] = -const-min_dis
#            
##    print('G:',G.shape,G)
##    print('h:',h.shape,h)                
#    G = matrix(G,(len(G),len(G[0])),'d')
#    h = matrix(h,(len(h),1),'d')
#    
#    # Equality constraints: fix vehicle's initial position
#    A = np.zeros((veh_num * dim, veh_num * horizon * dim + SV))
#    b = np.zeros((len(A), 1))
#           
#    for i in range(veh_num):
#        for j in range(dim):
#            A[i*dim + j][i*horizon*dim + j] = 1
#    
#            if SCCFS is True: # slack variables
#                A[i*dim + j][veh_num * horizon * dim + i * dim + j] = 1
#           
#    b = pos
#
##    print(A.shape)
##    print(b.shape) 
##    print('A:',A)
##    print('b:',b)         
#    A = matrix(A,(len(A),len(A[0])),'d')
#    b = matrix(b,(len(b),1),'d')    
#    
#    end = time.perf_counter()
#    print('Constraints:',end-start)
#    
#    # Solver        
#    sol = solvers.qp(P, q, G, h, A, b)
#    
#    end = time.perf_counter()
#    print('Solved time:',end-start)
#    
#    x_sol = sol['x']
#    if SCCFS is True: # slack variables
#        x_sol = sol['x'][:-SV]
#    x_sol = np.reshape(x_sol, (veh_num, horizon, dim))
#    
#    return x_sol





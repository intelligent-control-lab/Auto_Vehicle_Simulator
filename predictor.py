###################
# predictor.py
# this file predicts the trajectory of current vehicle
#
# Authors: Tianhao Wei
# Copyright: 2019
###################
from __future__ import division
from direct.stdpy.file import *
import math
import numpy as np
from cvxopt import matrix, solvers
import utility
import optimization as opt
import copy
import pickle
import sklearn
solvers.options['feastol'] = 1e-10

# this is a basic predictor that generate linear prediction
class basicPredictor:
    def __init__(self,dT):
        self.dT = dT * 5
        self.horizon = 10

    def setVehicle(self,vehicle):
        self.vehicle = vehicle
        self.sensor = vehicle.sensor
    
    def predict(self, v):
        traj = list()
        
        start = self.vehicle.getPosVector()
        for i in range(1,self.horizon):
            traj.append(start + v * i * self.dT)

        return traj
    
class HMMPredictor(basicPredictor):
    def __init__(self,dT):
        basicPredictor.__init__(self, dT)

        f = open('data/A.txt', 'rb')
        self.A = pickle.load(f)

        f = open('data/random_forest.txt', 'rb')
        self.random_forest = pickle.load(f)

        f = open('data/random_forest_changing.txt', 'rb')
        self.rfc = pickle.load(f)

        f = open('data/changing_lane_coef.txt', 'rb')
        self.clc = pickle.load(f)
        print(self.clc)

        print(self.A)

        self.P = np.array([0.1, 0.1, 0.8])
        self.B = 2
        self.avg_lateral_v = 0

    def calc_features(self, ngsim_vehicles, lane_width):
        
        ego = ngsim_vehicles[self.vehicle.id]
        vels = list()
        
        v_mean = ego.mean_vel
        
        if ego.Preceding in ngsim_vehicles.keys():
            front = ngsim_vehicles[ego.Preceding]
        else:
            front = None
        
        f = np.zeros(10)
        f[0] = lane_width
        f[1] = ego.v_Acc # because the angle is small, we use a to approximate longitudinal acceleration
        f[2] = 0 # No deceleration light info in data
        f[3] = ego.lateral_acc
        f[4] = ego.v_Vel - v_mean
        f[5] = ego.v_Vel - front.v_Vel if front is not None else 0
        f[6] = -1 if min(ego.Local_X % lane_width, lane_width - (ego.Local_X % lane_width)) - (ego.v_Width/2) < 0 else 0
        f[7] = 0
        f[8] = ego.lateral_vel # calculated by Local_X derivative
        f[9] = (ego.Local_X % lane_width) - (lane_width / 2)
        
        
        return f

    def measure_prob(self, f, d, cc):
        pred_chg = 1 / (1 + np.exp(cc[0] * abs(f[8]) + cc[1] * abs(f[9]) + cc[2]))
        pred_flw = 1 - pred_chg
        p = np.zeros(3)
        p[d] = pred_chg
        p[2] = pred_flw

        return p
    

    def sgn(x):
        return (x > 0) * 2 - 1     
    #TODO: set v to vehicle Class, rather than send it in
    def predict(self, v):
        
        f = self.calc_features(self.sensor.ngsim_vehicles, self.sensor.lane_width)
        
        traj = list()
        # 0 -- left, 1 -- right, 2 -- accelerate, 3 -- brake, 4 -- follow

        decay = (abs(f[9])/self.sensor.lane_width * 2)
        print(decay)
        self.avg_lateral_v = self.avg_lateral_v * decay + f[8]
        d = int(self.avg_lateral_v > 0)
        
        # turning = self.rfc.predict_proba(f[np.newaxis,:])[0]
        # PyB = np.zeros(3)
        # PyB[2] = turning[0]
        # PyB[d] = turning[1]
        PyB = self.measure_prob(f, d, self.clc)

        
        # if f[6] == -1:
        #     self.B = d
        #     PyB[self.B] = 1
        # else:
        #     self.P = np.multiply(PyB , np.matmul(self.P, self.A));
        #     self.P = self.P / np.sum(self.P)
        #     self.B = np.argmax(self.P)

        self.P = np.multiply(PyB , np.matmul(self.P, self.A));
        self.P = self.P / np.sum(self.P)
        self.B = np.argmax(self.P)

        # self.P = PyB
        # self.B = np.argmax(self.P)        
        

        v = np.array([0,0,0]) * 5
        if self.B == 0:
            v = np.array([-1,1,0]) * 5
        if self.B == 1:
            v = np.array([1,1,0]) * 5
        if self.B == 2:
            v = np.array([0,1,0]) * 5
        if self.B == 3:
            v = np.array([0,-1,0]) * 5
        if self.B == 4:
            v = np.array([0,0,0]) * 5

        start = self.vehicle.getPosVector() + np.array([0,0,5])
        for i in range(1,self.horizon):
            traj.append(start + v * i * self.dT)
        
        return (self.B, traj, self.P)

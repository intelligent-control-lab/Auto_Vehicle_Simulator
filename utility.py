####################
# utility.py
# this file specifis some useful functions in computation
#
# Author: Jianyu Chen
# Copyright: 2016
####################


import math
import numpy as np
from cvxopt import matrix

# Vector to Angle
def vec2ang(vector):
    x = vector[0]
    y = vector[1]
    return math.atan2(y,x)

# Angle to Vector
def ang2vec(theta):
    return [math.cos(theta),math.sin(theta)]

def perpLength(v1,v2):
    return np.cross(v2,v1)/np.linalg.norm(v2)

def normalize(v):
    norm = np.linalg(np.array(v))
    for i in range(len(v)):
        v[i] = v[i]/norm
    return v

# Minus of array
def substruct(v1,v2):
    v = []
    for i in range(len(v1)):
        v.append(v1[i]-v2[i])
    return v

# embedding a small matrix to a large matrix
def embed(Qaug, Q, ni, nj):
    [li,lj] =  Q.shape
    for i in range(ni,ni+li):
        for j in range(nj,nj+lj):
            Qaug[i][j] = Q[i-ni][j-nj]
    return Qaug

def matrixPower(A,n):
    if n == 0:
        return np.identity(math.sqrt(len(A)))
    else:
        newA = A
        for i in range(n-1):
            newA = newA*A
        return newA

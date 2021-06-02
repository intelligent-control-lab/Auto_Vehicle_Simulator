Auto Vehicle Simulator - CFS-based Distributed MPC
===

### About CFS-DMPC
This work is built on Auto Vehicle Simulator. We implement convex feasible set (CFS) algorithm in distributed MPC to solve multi-vehicle coordination efficiently. We propose a deadlock resolution by changing vehicle’s desire speed.


### Environment Setup with Anaconda
```
conda create -n avsim python=3.7
conda activate avsim
pip install numpy scipy matplotlib pandas sympy nose
pip install panda3d==1.10.3
conda install -c https://conda.anaconda.org/omnia cvxopt
```

### How to Run the Demo
Configure `self.scenario` in main.py for different scenarios. Use keyboard "c" to triger lane changing.


### Simulation
1. Crossing

![image](https://github.com/intelligent-control-lab/Auto_Vehicle_Simulator/blob/Distributed-CFS/SimulationResults/Panda3dSim/Corssing.gif)


2. Merging

![image](https://github.com/intelligent-control-lab/Auto_Vehicle_Simulator/blob/Distributed-CFS/SimulationResults/Panda3dSim/Merging.gif)


3. Platoon formation

![image](https://github.com/intelligent-control-lab/Auto_Vehicle_Simulator/blob/Distributed-CFS/SimulationResults/Panda3dSim/Platoon.gif)


4. Overtaking

![image](https://github.com/intelligent-control-lab/Auto_Vehicle_Simulator/blob/Distributed-CFS/SimulationResults/Panda3dSim/Overtaking.gif)


### References
H. Zhou, and C. Liu,  "Distributed motion coordination using convex feasible set algorithm", Preprint: https://arxiv.org/abs/2101.07994

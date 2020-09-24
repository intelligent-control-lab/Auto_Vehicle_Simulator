Auto Vehicle Simulator - CFS-based Distributed MPC
===

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


### Scenarios
0. Overtaking
1. Crossing
2. Platoon formation
3-5. Merging


### References
H. Zhou, and C. Liu,  "Distributed motion coordination using convex feasible set algorithm"
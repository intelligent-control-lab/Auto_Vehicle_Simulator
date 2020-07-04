Auto Vehicle Simulator - multicar
===
This "multicar" branch is an application built on Auto Vehicle Simulator, which demonstrates the Multi-car Convex Feasible Set algorithm. There are 2 built-in scenarios for display, a 2-car overtake and a crowded 9-car plot.

### Environment Setup with Anaconda
```
conda create -n avsim python=3.6
conda activate avsim
pip install numpy scipy matplotlib pandas sympy nose
pip install panda3d==1.10.3
conda install -c https://conda.anaconda.org/omnia cvxopt
pip install cvxpy
```

### How to Run the Demo
1. Planning mode
Configure `self.scenario` in main.py for different scenarios, leave `self.replayFile` as `None` for this mode. Use keyboard "c" to triger lane changing.
2. Replay mode
Configure `self.replayFile` to a trajectories log file. The simulator will use the saved trajectories without live planning.

### Scenarios
1. 2-car overtake

![](https://i.imgur.com/1yU8rf8.png)

logged filename: "traj_log_2.npz"

2. crowded 9-car

![](https://i.imgur.com/GTsUAxt.png)

logged filename: "traj_log_9.npz"

### References
Huang, J., and Liu, C., 2020 "Multi-car Convex Feasible Set Algorithm in Trajectory Planning"

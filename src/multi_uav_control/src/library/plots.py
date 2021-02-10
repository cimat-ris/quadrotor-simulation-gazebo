import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from formation_plots import *

path = "/home/cimat/bebop_ws/src/multi_uav_control/output/"

# Agents positions
x = np.loadtxt(path+"data/q_odom.dat", skiprows=1)
# Virtual agents
z = np.loadtxt(path+"data/qz_agents.dat", skiprows=1)
# Consensus error
e = np.loadtxt(path+"data/e_consensus.dat", skiprows=1)
# Computed controls
#comp_u = np.loadtxt(path+"data/qp_computed_controls.dat", skiprows=1)
n = int(x.shape[0]/3)
# Control inputs
uList = []
for i in range(n):
	u = np.loadtxt(path+"data/qp_agent_"+str(i)+".dat", skiprows=1)
	uList.append(u)

column = {}
column['desc'] = "column"
column['x'] = 0.5
column['y'] = 0.5

plot_3d_formation(x, path=path+"plots/")
plot_2d_formation(x, path=path+"plots/")
plot_3d_virtual(z, path=path+"plots/")
plot_virtual_evolution(z, path=path+"plots/")
plot_controls_u(uList, path=path+"plots/", minu=-5.0, maxu=5.0)
plot_consensus_error(e, path=path+"plots/")
plot_distances(x, D=0.5, obsF=[column], mind=0.0, maxd=2.5)

print("Plots generated in "+path+"plots/")
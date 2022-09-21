import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.io as sio
import control as ctr


from casadi import *
from casadi.tools import *


import sys
sys.path.append('./sources')
from System import System, def_system
from Plots import plot_lqr, custom_plot, plot_cl_wo_noise, plot_spc_vs_deepc
from Data_Collection import random_u, data_collection
from Optim import optim_DeePC, DeePC_cl_wo_noise, optim_SPC, SPC_cl_wo_noise


#---------------------- Initialization ----------------------#

# Random seed:
np.random.seed(12)

custom_plot()

m = 28e-3
T_ini = 10
Tf = 25
Td = 150
L = T_ini + Tf

n_u = 3
n_y = 5

sig_w = 0

x_r = np.array([[1],[-2],[-1.5],[0],[0],[0],[0],[0]])
y_r = np.array([[1],[-2],[-1.5],[0],[0]])


#---------------------- System Definition ----------------------#
[A_cl,B_cl,C_cl, K1] = def_system(m,1)
sys = System(x_r,K1,1,A_cl,B_cl,C_cl)

sys.reset()

# u0 = np.zeros((3,1))
# for k in range(1000):
#     # if k<20:
#     #     u0 = np.array([[m*9.81],[0],[0]])
#     # else:
#     #     u0 = np.array([[0],[0],[0]])
#     u0 = np.array([[0],[0],[0]])
#     sys.make_step(u0)

# plot_lqr(sys.time, sys.u, sys.y)



#---------------------- Data Collection ----------------------#
u0 = np.zeros((3,1))
U_L, Y_L = data_collection(u0, sys, Td, L, 0, sig_w)

U_p, U_f = np.split(U_L, [n_u*T_ini],axis=0)
Y_p, Y_f = np.split(Y_L, [n_y*T_ini],axis=0)


#---------------- Building the Optimizer: DeePC ----------------#

Q = 10*np.identity(5)
R = np.array([[160, 0, 0],[0, 2, 0],[0, 0, 2]])


S_deepc, opt_x_num_dpc, opt_p_num_dpc = optim_DeePC(y_r, Q, R, Tf, Td, T_ini, \
                                                    U_p, Y_p, Y_f, U_f, n_u, n_y)

res_deePC, y_optim_deePC, g = DeePC_cl_wo_noise(sys, T_ini,Td, opt_p_num_dpc,\
                                          opt_x_num_dpc, S_deepc, y_r, R, Q)

    
# plot_cl_wo_noise(res_deePC['time'], res_deePC['u'], res_deePC['y'], y_r, \
#             res_deePC['cost'], y_optim)

    
#----------------- Building the Optimizer: SPC -----------------#

Q = 10*np.identity(5)
R = np.array([[160, 0, 0],[0, 2, 0],[0, 0, 2]])


S_spc, opt_x_num_spc, opt_p_num_spc = optim_SPC(y_r, Q, R, Tf, Td, T_ini, U_p, \
                                                Y_p, Y_f, U_f, n_u, n_y)

res_SPC, y_optim_spc = SPC_cl_wo_noise(sys, T_ini,Td, opt_p_num_spc,\
                                          opt_x_num_spc, S_spc, y_r, R, Q)

    
# plot_cl_wo_noise(res_SPC['time'], res_SPC['u'], res_SPC['y'], y_r, \
#             res_SPC['cost'], y_optim_spc)

#---------------- Plots: SPC vs DeePC ----------------#  

plot_spc_vs_deepc(res_SPC['time'], res_SPC['u'], res_SPC['y'], y_r, res_SPC['cost'], \
                  y_optim_spc, res_deePC['time'], res_deePC['u'], res_deePC['y'], \
                  res_deePC['cost'], y_optim_deePC)
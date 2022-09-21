import numpy as np
from casadi import *
from casadi.tools import *

from Data_Collection import random_u


def optim_DeePC(y_r, Q, R, Tf, Td, T_ini, U_p, Y_p, Y_f, U_f, n_u, n_y):
    
    opt_x_dpc = struct_symMX([
        entry('g', shape=(Td)),
        entry('u_f', shape=(n_u), repeat=Tf),
        entry('y_f', shape=(n_y), repeat=Tf)
    ])
    
    opt_p_dpc = struct_symMX([
        entry('u_ini', shape=(n_u), repeat=T_ini),
        entry('y_ini', shape=(n_y), repeat=T_ini),
    ])
    
    # Create numerical instances of the structures (holding all zeros as entries)
    opt_x_num_dpc = opt_x_dpc(0)
    opt_p_num_dpc = opt_p_dpc(0)
    
    # Create the objective:
    obj = 0
    for k in range(Tf):    
        obj += (opt_x_dpc['y_f',k] - y_r).T@(Q)@(opt_x_dpc['y_f',k] - y_r) + \
                (opt_x_dpc['u_f',k]).T@(R)@(opt_x_dpc['u_f',k])
    
        
    # Create the constraints
    A = vertcat(U_p, Y_p, U_f, Y_f)
    b = vertcat(*opt_p_dpc['u_ini'], *opt_p_dpc['y_ini'], *opt_x_dpc['u_f'], *opt_x_dpc['y_f'])
    
    
    cons = A@opt_x_dpc['g']-b
    
    # Create lower and upper bound structures and set all values to plus/minus infinity.
    lbx_dpc = opt_x_dpc(-np.inf)
    ubx_dpc = opt_x_dpc(np.inf)
    
    
    # Create Optim
    nlp = {'x':opt_x_dpc, 'f':obj, 'g':cons, 'p':opt_p_dpc}
    S_deepc = nlpsol('S', 'ipopt', nlp)
    
    return S_deepc, opt_x_num_dpc, opt_p_num_dpc



def optim_SPC(y_r, Q, R, Tf, Td, T_ini, U_p, Y_p, Y_f, U_f, n_u, n_y):
    
    M = np.concatenate((Y_p, U_p, U_f))
    P = Y_f@np.linalg.pinv(M)
    
    opt_x_spc = struct_symMX([
        entry('u_f', shape=(n_u), repeat=Tf),
        entry('y_f', shape=(n_y), repeat=Tf)
    ])
    
    opt_p_spc = struct_symMX([
        entry('u_ini', shape=(n_u), repeat=T_ini),
        entry('y_ini', shape=(n_y), repeat=T_ini),
    ])
    
    # Create numerical instances of the structures (holding all zeros as entries)
    opt_x_num_spc = opt_x_spc(0)
    opt_p_num_spc = opt_p_spc(0)
    
    
    # Create the objective:
    obj = 0
    for k in range(Tf):    
        obj += (opt_x_spc['y_f',k] - y_r).T@(Q)@(opt_x_spc['y_f',k] - y_r) + \
                (opt_x_spc['u_f',k]).T@(R)@(opt_x_spc['u_f',k])
    
        
    # Create the constraints:
    b = vertcat(*opt_p_spc['y_ini'], *opt_p_spc['u_ini'], *opt_x_spc['u_f'])
    y_f = vertcat(*opt_x_spc['y_f'])
    cons = P@b-y_f
    
    
    # Create lower and upper bound structures and set all values to plus/minus infinity.
    lbx_dpc = opt_x_spc(-np.inf)
    ubx_dpc = opt_x_spc(np.inf)
    
    
    # Create Optim
    nlp = {'x':opt_x_spc, 'f':obj, 'g':cons, 'p':opt_p_spc}
    S_spc = nlpsol('S', 'ipopt', nlp)
    
    return S_spc, opt_x_num_spc, opt_p_num_spc


def DeePC_cl_wo_noise(sys, T_ini, Td, opt_p_num_dpc, opt_x_num_dpc, S_deepc,\
                      y_r, R, Q):
    
    sys.reset()
    
    n_ciclos = 300

    # Excitement  
    n_exc = T_ini
    u0 = np.zeros((3,1))
    for k in range(n_exc):
        u0 = random_u(u0)
        sys.make_step(u0)
              
    # Control
    cost = []
    y_optim = np.zeros((n_ciclos,5))
    g_optim = np.zeros((n_ciclos,Td))
    for k in range(n_ciclos):
        u0 = np.zeros((3,1))
        
        y_Tini = sys.y[-T_ini:,:]
        u_Tini = sys.u[-T_ini:,:] 
    
        opt_p_num_dpc['y_ini'] = vertsplit(y_Tini)
        opt_p_num_dpc['u_ini'] = vertsplit(u_Tini)
        r = S_deepc(p=opt_p_num_dpc, lbg=0, ubg=0)
        opt_x_num_dpc.master = r['x']
        u0 = opt_x_num_dpc['u_f',0].full()
        
        y_optim[k] = opt_x_num_dpc['y_f',0].full().T
        g_optim[k] = opt_x_num_dpc['g'].full().T
        
        
        y0 = sys.make_step(u0)
               
        cost.append((y0-y_r).T@Q@(y0-y_r) + u0.T@R@u0)
        
        print("------------ ",k," de ",n_ciclos," CICLOS ---------")
        
    res_DeePC = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}

    
    return res_DeePC, y_optim, g_optim



def SPC_cl_wo_noise(sys, T_ini, Td, opt_p_num_spc, opt_x_num_spc, S_spc,\
                      y_r, R, Q):
    
    sys.reset()
    
    n_ciclos = 300

    # Excitement  
    n_exc = T_ini
    u0 = np.zeros((3,1))
    for k in range(n_exc):
        u0 = random_u(u0)
        sys.make_step(u0)
              
    # Control
    cost = []
    y_optim = np.zeros((n_ciclos,5))
    for k in range(n_ciclos):
    
        y_Tini = sys.y[-T_ini:,:]
        u_Tini = sys.u[-T_ini:,:] 
    
        opt_p_num_spc['y_ini'] = vertsplit(y_Tini)
        opt_p_num_spc['u_ini'] = vertsplit(u_Tini)
        r = S_spc(p=opt_p_num_spc, lbg=0, ubg=0)
        opt_x_num_spc.master = r['x']
        u0 = opt_x_num_spc['u_f',0].full()
        
        y_optim[k] = opt_x_num_spc['y_f',0].full().T
        
        
        y0 = sys.make_step(u0)
               
        cost.append((y0-y_r).T@Q@(y0-y_r) + u0.T@R@u0)
        
    res_SPC = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}

    
    return res_SPC, y_optim
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


def custom_plot():
    # Customizing Matplotlib:
    mpl.rcParams['font.size'] = 12
    mpl.rcParams['figure.figsize'] = [10, 5]
    mpl.rcParams['lines.linewidth'] = 2
    mpl.rcParams['axes.grid'] = True
    mpl.rcParams['svg.fonttype'] = 'none'
    mpl.rcParams['axes.unicode_minus'] = 'true'
    mpl.rcParams['axes.labelsize'] = 'medium'
    mpl.rcParams['legend.fontsize'] = 'medium'
    mpl.rcParams['xtick.labelsize'] = 'medium'
    mpl.rcParams['ytick.labelsize'] = 'medium'
    mpl.rcParams['axes.labelpad'] = 3

def plot_lqr(time, u, y):
    
    plt.figure(1)
    plt.plot(time,y[:,0])
    plt.xlabel("Time [s]")
    plt.ylabel("x [m]")
    
    plt.figure(2)
    plt.plot(time,y[:,1])
    plt.xlabel("Time [s]")
    plt.ylabel("y [m]")
    
    plt.figure(3)
    plt.plot(time,-y[:,2])
    plt.xlabel("Time [s]")
    plt.ylabel("h [m]")
    
    plt.figure(4)
    plt.plot(time,y[:,3]*180/np.pi)
    plt.xlabel("Time [s]")
    plt.ylabel("roll [degree]")
    
    plt.figure(5)
    plt.plot(time,y[:,4]*180/np.pi)
    plt.xlabel("Time [s]")
    plt.ylabel("pitch [degree]")
        
    plt.figure(6)
    plt.plot(time,u)
    plt.xlabel("Time [s]")
    plt.ylabel("Magnitude")
    plt.legend(['Delta_T', 'w_x', 'w_y'])
    
    
    
def plot_cl_wo_noise(time, u, y, y_r, cost, y_optim):
    
    y_r0 = y_r[0]*np.ones((np.size(time),1))
    y_r1 = y_r[1]*np.ones((np.size(time),1))
    y_r2 = y_r[2]*np.ones((np.size(time),1))
    y_r3 = y_r[3]*np.ones((np.size(time),1))
    y_r4 = y_r[4]*np.ones((np.size(time),1))

    
    plt.figure(1)
    plt.plot(time,y[:,0], label='model')
    plt.plot(time,y_optim[:,0], label='optim')
    plt.plot(time,y_r0, label='ref')
    plt.xlabel("Time [s]")
    plt.ylabel("x [m]")
    plt.legend(['model', 'optim', 'ref'])
    
    
    plt.figure(2)
    plt.plot(time,y[:,1], label='model')
    plt.plot(time,y_optim[:,1], label='optim')
    plt.plot(time,y_r1, label='ref')
    plt.xlabel("Time [s]")
    plt.ylabel("y [m]")
    plt.legend(['model', 'optim', 'ref'])
    
    
    plt.figure(3)
    plt.plot(time,-y[:,2], label='model')
    plt.plot(time,-y_optim[:,2], label='optim')
    plt.plot(time,-y_r2, label='ref')
    plt.xlabel("Time [s]")
    plt.ylabel("h [m]")
    plt.legend(['model', 'optim', 'ref'])
    
    
    plt.figure(4)
    plt.plot(time,y[:,3]*180/np.pi, label='model')
    plt.plot(time,y_optim[:,3]*180/np.pi, label='optim')
    plt.plot(time,y_r3*180/np.pi, label='ref')
    plt.xlabel("Time [s]")
    plt.ylabel("roll [degree]")
    plt.legend(['model', 'optim', 'ref'])
    
    
    plt.figure(5)
    plt.plot(time,y[:,4]*180/np.pi, label='model')
    plt.plot(time,y_optim[:,4]*180/np.pi, label='optim')
    plt.plot(time,y_r4*180/np.pi, label='ref')
    plt.xlabel("Time [s]")
    plt.ylabel("pitch [degree]")
    plt.legend(['model', 'optim', 'ref'])
    
    
    plt.figure(6)
    plt.plot(time,u)
    plt.xlabel("Time [s]")
    plt.ylabel("Magnitude")
    plt.legend(['Delta_T', 'w_x', 'w_y'])
    
    plt.figure(7)
    plt.plot(time,cost)
    plt.xlabel("Time [s]")
    plt.ylabel("Cost")
    
    plt.show()
    
    
def plot_spc_vs_deepc(time_spc, u_spc, y_spc, y_r, cost_spc, y_optim_spc,\
                      time_deepc, u_deepc, y_deepc, cost_deepc, y_optim_deepc):
    
    y_r0_deepc = y_r[0]*np.ones((np.size(time_deepc),1))
    y_r1_deepc = y_r[1]*np.ones((np.size(time_deepc),1))
    y_r2_deepc = y_r[2]*np.ones((np.size(time_deepc),1))
    y_r3_deepc = y_r[3]*np.ones((np.size(time_deepc),1))
    y_r4_deepc = y_r[4]*np.ones((np.size(time_deepc),1))
    
    y_r0_spc = y_r[0]*np.ones((np.size(time_spc),1))
    y_r1_spc = y_r[1]*np.ones((np.size(time_spc),1))
    y_r2_spc = y_r[2]*np.ones((np.size(time_spc),1))
    y_r3_spc = y_r[3]*np.ones((np.size(time_spc),1))
    y_r4_spc = y_r[4]*np.ones((np.size(time_spc),1))
    
    #---------------------- x ----------------------#
    
    fig1, ax1 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax1[0].set_title('DeePC')
    ax1[0].plot(time_deepc, y_deepc[:,0], label = 'model')
    ax1[0].plot(time_deepc, y_optim_deepc[:,0], label = 'optim')
    ax1[0].plot(time_deepc, y_r0_deepc, label = 'ref')
    
    ax1[1].set_title('SPC')
    ax1[1].plot(time_spc, y_spc[:,0], label = 'model')
    ax1[1].plot(time_spc, y_optim_spc[:,0], label = 'optim')
    ax1[1].plot(time_spc, y_r0_spc, label = 'ref')
    
    ax1[0].set_ylabel('x [m]')
    
    ax1[1].legend(['model', 'optim', 'ref'])
    
    ax1[0].set_xlabel('time [s]')
    ax1[1].set_xlabel('time [s]')
    
    fig1.align_ylabels()
    fig1.align_xlabels()
    fig1.tight_layout()
    plt.show()
    
    #---------------------- y ----------------------#
    
    fig2, ax2 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax2[0].set_title('DeePC')
    ax2[0].plot(time_deepc, y_deepc[:,1], label = 'model')
    ax2[0].plot(time_deepc, y_optim_deepc[:,1], label = 'optim')
    ax2[0].plot(time_deepc, y_r1_deepc, label = 'ref')
    
    ax2[1].set_title('SPC')
    ax2[1].plot(time_spc, y_spc[:,1], label = 'model')
    ax2[1].plot(time_spc, y_optim_spc[:,1], label = 'optim')
    ax2[1].plot(time_spc, y_r1_spc, label = 'ref')
    
    ax2[0].set_ylabel('y [m]')
    
    ax2[1].legend(['model', 'optim', 'ref'])
    
    ax2[0].set_xlabel('time [s]')
    ax2[1].set_xlabel('time [s]')
    
    fig2.align_ylabels()
    fig2.align_xlabels()
    fig2.tight_layout()
    plt.show()
    
    #---------------------- h ----------------------#
    
    fig3, ax3 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax3[0].set_title('DeePC')
    ax3[0].plot(time_deepc, -y_deepc[:,2], label = 'model')
    ax3[0].plot(time_deepc, -y_optim_deepc[:,2], label = 'optim')
    ax3[0].plot(time_deepc, -y_r2_deepc, label = 'ref')
    
    ax3[1].set_title('SPC')
    ax3[1].plot(time_spc, -y_spc[:,2], label = 'model')
    ax3[1].plot(time_spc, -y_optim_spc[:,2], label = 'optim')
    ax3[1].plot(time_spc, -y_r2_spc, label = 'ref')
    
    ax3[0].set_ylabel('h [m]')
    
    ax3[1].legend(['model', 'optim', 'ref'])
    
    ax3[0].set_xlabel('time [s]')
    ax3[1].set_xlabel('time [s]')
    
    fig3.align_ylabels()
    fig3.align_xlabels()
    fig3.tight_layout()
    plt.show()
    
    #---------------------- roll ----------------------#
    
    fig4, ax4 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax4[0].set_title('DeePC')
    ax4[0].plot(time_deepc, y_deepc[:,3]*180/np.pi, label = 'model')
    ax4[0].plot(time_deepc, y_optim_deepc[:,3]*180/np.pi, label = 'optim')
    ax4[0].plot(time_deepc, y_r3_deepc*180/np.pi, label = 'ref')
    
    ax4[1].set_title('SPC')
    ax4[1].plot(time_spc, y_spc[:,3]*180/np.pi, label = 'model')
    ax4[1].plot(time_spc, y_optim_spc[:,3]*180/np.pi, label = 'optim')
    ax4[1].plot(time_spc, y_r3_spc*180/np.pi, label = 'ref')
    
    ax4[0].set_ylabel('roll [degree]')
    
    ax4[1].legend(['model', 'optim', 'ref'])
    
    ax4[0].set_xlabel('time [s]')
    ax4[1].set_xlabel('time [s]')
    
    fig4.align_ylabels()
    fig4.align_xlabels()
    fig4.tight_layout()
    plt.show()
    
    #---------------------- pitch ----------------------#
    
    fig5, ax5 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax5[0].set_title('DeePC')
    ax5[0].plot(time_deepc, y_deepc[:,4]*180/np.pi, label = 'model')
    ax5[0].plot(time_deepc, y_optim_deepc[:,4]*180/np.pi, label = 'optim')
    ax5[0].plot(time_deepc, y_r4_deepc*180/np.pi, label = 'ref')
    
    ax5[1].set_title('SPC')
    ax5[1].plot(time_spc, y_spc[:,4]*180/np.pi, label = 'model')
    ax5[1].plot(time_spc, y_optim_spc[:,4]*180/np.pi, label = 'optim')
    ax5[1].plot(time_spc, y_r4_spc*180/np.pi, label = 'ref')
    
    ax5[0].set_ylabel('pitch [degree]')
    
    ax5[1].legend(['model', 'optim', 'ref'])
    
    ax5[0].set_xlabel('time [s]')
    ax5[1].set_xlabel('time [s]')
    
    fig5.align_ylabels()
    fig5.align_xlabels()
    fig5.tight_layout()
    plt.show()
    
    #---------------------- inputs ----------------------#
    
    fig6, ax6 = plt.subplots(1,2,sharex=True, sharey='row')
    
    ax6[0].set_title('DeePC')
    ax6[0].plot(time_deepc, u_deepc)
    
    ax6[1].set_title('SPC')
    ax6[1].plot(time_spc, u_spc)
    
    ax6[0].set_ylabel('magnitude')
    
    ax6[1].legend(['delta T', 'wx', 'wy'])
    
    ax6[0].set_xlabel('time [s]')
    ax6[1].set_xlabel('time [s]')
    
    fig6.align_ylabels()
    fig6.align_xlabels()
    fig6.tight_layout()
    plt.show()
    
    #---------------------- cost ----------------------#
    
    fig7, ax7 = plt.subplots(1,2,sharex=True)
    
    ax7[0].set_title('DeePC')
    ax7[0].plot(time_deepc, cost_deepc)
    
    ax7[1].set_title('SPC')
    ax7[1].plot(time_spc, cost_spc)
    
    ax7[0].set_ylabel('cost')
    ax7[1].set_ylabel('cost')
    
    
    ax7[0].set_xlabel('time [s]')
    ax7[1].set_xlabel('time [s]')
    
    fig7.align_ylabels()
    fig7.align_xlabels()
    fig7.tight_layout()
    plt.show()
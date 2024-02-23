import sys

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
sys.path.append('../')
from landing_controller.controller.utility import *

np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)
import os
from scipy.io import loadmat

#directory = os.environ['LOCOSIM_DIR'] + '/landing_controller/experiments/20230421_145931/'
directory = os.environ['LOCOSIM_DIR'] + '/landing_controller/simulations/ELCtests/20231128_170343/'
filename = "DATA.mat"
DATA = loadmat(directory + filename)
# Convert dictionary entries into local variables
# locals().update(DATA)

time_log = DATA['time_log'].flatten()
apex_sample0 = DATA['apex_sample0'].item()
touch_down_sample0 = DATA['touch_down_sample0'].item()
lift_off_sample1 = 281#DATA['lift_off_sample1'].item()
apex_sample1 = DATA['apex_sample1'].item()
touch_down_sample1 = DATA['touch_down_sample1'].item()

comPoseW_des_log = DATA['comPoseW_des_log']
comPoseW_log = DATA['comPoseW_log']

tau_des_log = DATA['tau_des_log']
tau_log = DATA['tau_log']

grForcesW_log = DATA['grForcesW_log']
grForcesW_des_log = DATA['grForcesW_des_log']
grForcesW_wbc_log = DATA['grForcesW_wbc_log']

q_des_log = DATA['q_des_log']
q_log = DATA['q_log']
qd_des_log = DATA['qd_des_log']
qd_log = DATA['qd_log']

del DATA
from base_controllers.utils.common_functions import *
#fig = plotJoint('torque', time_log=time_log, tau_log=tau_log,sharex=True, sharey=False, start=apex_sample0, end=-1)
# fig = plotJoint('velocity', time_log=time_log, qd_log=qd_log, sharex=True, sharey=False, start=apex_sample0, end=-1)
# start = int(21.2/0.002)
# end = int(21.5/0.002)
# print(np.std(tau_log[8, start:end]))
# print(np.std(qd_log[8, start:end]))

sharex=True
sharey=False
start=apex_sample0
end=-1
last = 972


import matplotlib as mpl
from matplotlib.lines import Line2D
mpl.use('TkAgg')

size_font = 12
mpl.rcdefaults()
mpl.rcParams['lines.linewidth'] = 2
#mpl.rcParams['lines.markersize'] = 6
#mpl.rcParams['patch.linewidth'] = 4
mpl.rcParams['axes.grid'] = True
mpl.rcParams['axes.labelsize'] = size_font
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.size'] = size_font
mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
                              'New Century Schoolbook',
                              'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
                              'Palatino',
                              'Charter', 'serif']
mpl.rcParams['text.usetex'] = True
mpl.rcParams['legend.fontsize'] = size_font - 2
plt.rcParams['legend.title_fontsize'] = size_font - 2
mpl.rcParams['legend.loc'] = 'best'
mpl.rcParams['figure.facecolor'] = 'white'
mpl.rcParams['figure.figsize'] = 9.2, 10.2
mpl.rcParams['savefig.format'] = 'pdf'
# mpl.rcParams['mathtext.fontset'] = 'dejavuserif'
# mpl.rcParams['mathtext.bf'] = 'serif:bold'
plt.rcParams.update(
    {
        "text.usetex": True,
        "text.latex.preamble": r"\usepackage{bm}",

        # Enforce default LaTeX font.
        # "font.family": "serif",
        "font.serif": ["Computer Modern"],
    }
)

from  base_controllers.utils.common_functions import *
fig = plt.figure(4)

ax1 = plt.subplot2grid((3, 2), (0, 0), colspan=2)
ax1.set_ylabel(r'pose [m], [rad]')
ax1.set_xlabel(r"time [s]")
ax1.set_xlim([-0.02, 2.02])
plot1= []

#### desired pose
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = comPoseW_des_log[0, start:end]
graph['label'] = r'$c^x_d$'
graph['color'] = 'red'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = comPoseW_des_log[2, start:end]
graph['label'] = r'$c^z_d$'
graph['color'] = 'darkgreen'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)
#pitch
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = comPoseW_des_log[4, start:end]
graph['label'] = r'pitch$_d$'
graph['color'] = 'blue'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)

# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
#graph['y'] = np.hstack([comPoseW_log[0, apex_sample0:touch_down_sample0],  1.5*comPoseW_log[0, last] - 0.5*comPoseW_log[0, touch_down_sample0+1:]])
graph['y'] = comPoseW_log[0, start:end]#,  1.5*comPoseW_log[0, last] - 0.5*comPoseW_log[0, touch_down_sample0+1:]])
graph['label'] = r'$c^x$'
graph['color'] = 'tomato'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot1.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
#graph['y'] = np.hstack([comPoseW_log[2, apex_sample0:touch_down_sample0],  comPoseW_log[2, last] + 0.7*(comPoseW_log[2, touch_down_sample0+1:]-comPoseW_log[2, last]) ])
graph['y'] = comPoseW_log[2, start:end]
graph['label'] = r'$c^z$'
graph['color'] = 'limegreen'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot1.append(graph)
#pitch
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = comPoseW_log[4, start:end]
graph['label'] = r'pitch'
graph['color'] = 'cornflowerblue'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot1.append(graph)

for graph in plot1:
    ax1.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])

ax1.set_ylim(ax1.get_ylim())
ax1.vlines(touch_down_sample0*time_log[1]-time_log[start], ax1.get_ylim()[0], ax1.get_ylim()[1], linestyles='solid', zorder=10)
ax1.vlines(touch_down_sample1*time_log[1]-time_log[start], ax1.get_ylim()[0], ax1.get_ylim()[1], linestyles='solid', zorder=10)
ax1.vlines(lift_off_sample1*time_log[1]-time_log[start], ax1.get_ylim()[0], ax1.get_ylim()[1], linestyles='dashed', zorder=10)

legend_elements = [Line2D([0], [0], color=graph['color'], linestyle=graph['linestyle'], linewidth=graph['legend_linewidth'],
                          label=graph['label']) for graph in plot1]
ax1.legend(handles=legend_elements, bbox_to_anchor=(0.79,0.5), handlelength=1.6, handleheight=1.5, labelspacing=0.05, ncol=2,
          columnspacing=0.8)



grForcesW_des_log[:, :touch_down_sample0] = 0
grForcesW_log[:, :apex_sample0] *= 0.5

ax2 = plt.subplot2grid((3, 2), (1, 0), colspan=1)
ax2.set_ylabel(r'lf grf [N]')
ax2.set_xlabel(r"time [s]")

grForcesW_des_log[2, start:end] = np.clip(grForcesW_des_log[2, start:end], -np.inf, 180)
grForcesW_des_log[:, start:touch_down_sample0] = np.nan
grForcesW_des_log[:, lift_off_sample1:touch_down_sample1] = np.nan
grForcesW_log[2, start+50:end] = np.clip(grForcesW_log[2, start+50:end], 0, +np.inf)

plot2 = []

#### desired lf grf
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[0, start:end]
graph['label'] = r'$f^x_{lf, \, d}$'
graph['color'] = 'red'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[1, start:end]
graph['label'] = r'$f^y_{lf, \, d}$'
graph['color'] = 'darkgreen'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[2, start:end]
graph['label'] = r'$f^z_{lf, \, d}$'
graph['color'] = 'blue'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)

####  lf grf
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[0, start:end]
graph['label'] = r'$\hat{f}^x_{lf}$'
graph['color'] = 'tomato'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot2.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[1, start:end]
graph['label'] = r'$\hat{f}^y_{lf}$'
graph['color'] = 'limegreen'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot2.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[2, start:end]
graph['label'] = r'$\hat{f}^z_{lf}$'
graph['color'] = 'cornflowerblue'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot2.append(graph)


for graph in plot2:
    ax2.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])

legend_elements = [Line2D([0], [0], color=graph['color'], linestyle=graph['linestyle'], linewidth=graph['legend_linewidth'],
                          label=graph['label']) for graph in plot2]
ax2.legend(handles=legend_elements, loc='upper right', handlelength=1.6, handleheight=1.5, labelspacing=0.05, ncol=2,
          columnspacing=0.8)
ax2.set_xticks(np.arange(0, 2.1, step=0.5))
ax2.set_yticks(np.arange(-200, 310, step=100))
ax2.set_xlim([-0.05, 2.05])
ax2.set_ylim([-220,320])
ax2.vlines(touch_down_sample0*time_log[1]-time_log[start], ax2.get_ylim()[0], ax2.get_ylim()[1], linestyles='solid', zorder=10)
ax2.vlines(touch_down_sample1*time_log[1]-time_log[start], ax2.get_ylim()[0], ax2.get_ylim()[1], linestyles='solid', zorder=10)
ax2.vlines(lift_off_sample1*time_log[1]-time_log[start], ax2.get_ylim()[0], ax2.get_ylim()[1], linestyles='dashed', zorder=10)


ax3 = plt.subplot2grid((3, 2), (1, 1), colspan=1)
ax3.set_ylabel(r'lh grf [N]')
ax3.set_xlabel(r"time [s]")
leg = 1


plot3 = []

#### desired lh grf
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[3, start:end]
graph['label'] = r'$f^x_{lh, \, d}$'
graph['color'] = 'red'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[4, start:end]
graph['label'] = r'$f^y_{lh, \, d}$'
graph['color'] = 'darkgreen'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_des_log[5, start:end]
graph['label'] = r'$f^z_{lh, \, d}$'
graph['color'] = 'blue'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)

####  lh grf
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[3, start:end]
graph['label'] = r'$\hat{f}^x_{lh}$'
graph['color'] = 'tomato'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot3.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[4, start:end]
graph['label'] = r'$\hat{f}^y_{lh}$'
graph['color'] = 'limegreen'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot3.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = grForcesW_log[5, start:end]
graph['label'] = r'$\hat{f}^z_{lh}$'
graph['color'] = 'cornflowerblue'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot3.append(graph)

for graph in plot3:
    ax3.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])

legend_elements = [Line2D([0], [0], color=graph['color'], linestyle=graph['linestyle'], linewidth=graph['legend_linewidth'],
                          label=graph['label']) for graph in plot3]
ax3.legend(handles=legend_elements, loc='upper right', handlelength=1.6, handleheight=1.8, labelspacing=0.05, ncol=2,
          columnspacing=1)

ax3.set_xticks(np.arange(0, 2.1, step=0.5))
ax3.set_yticks(np.arange(-200, 310, step=100))
ax3.set_xlim([-0.05, 2.05])
ax3.set_ylim([-220,320])
ax3.vlines(touch_down_sample0*time_log[1]-time_log[start], ax3.get_ylim()[0], ax3.get_ylim()[1], linestyles='solid', zorder=10)
ax3.vlines(touch_down_sample1*time_log[1]-time_log[start], ax3.get_ylim()[0], ax3.get_ylim()[1], linestyles='solid', zorder=10)
ax3.vlines(lift_off_sample1*time_log[1]-time_log[start], ax3.get_ylim()[0], ax3.get_ylim()[1], linestyles='dashed', zorder=10)

# sublpot torques
ax4 = ax2 = plt.subplot2grid((3, 2), (2,0), colspan=2)
ax4.set_ylabel(r"torques [Nm]")
ax4.set_xlabel(r"time [s]")

fig.align_ylabels(fig.axes)
fig.tight_layout()

plot4=[]
#### lh torques
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[0, start:end]
graph['label'] = r'$\tau_{lf, \, haa}$'
graph['color'] = 'red'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[1, start:end]
graph['label'] = r'$\tau_{lf, \, hfe}$'
graph['color'] = 'darkgreen'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[2, start:end]
graph['label'] = r'$\tau_{lf, \, kfe}$'
graph['color'] = 'blue'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# tau_max_haa_hfe = 23.7
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = 23.7 * np.ones_like(time_log[start:end])
graph['label'] = r'$\tau_{ub, haa}$'
graph['color'] = 'grey'
graph['linestyle'] = '--'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)

####  lh grf
# x
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[3, start:end]
graph['label'] = r'$\tau_{lh, \, haa}$'
graph['color'] = 'tomato'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# y
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[4, start:end]
graph['label'] =r'$\tau_{lh, \, hfe}$'
graph['color'] = 'limegreen'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# z
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = tau_log[5, start:end]
graph['label'] = r'$\tau_{lh, \, kfe}$'
graph['color'] = 'cornflowerblue'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)
# tau_max_kfe = 35.55
graph = {}
graph['x'] = time_log[start:end]-time_log[start]
graph['y'] = 35.55 * np.ones_like(time_log[start:end])
graph['label'] = r'$\tau_{ub, hfe / kfe}$'
graph['color'] = 'grey'
graph['linestyle'] = '-'
graph['linewidth'] = 2
graph['legend_linewidth'] = 2
plot4.append(graph)


for graph in plot4:
    ax4.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])

legend_elements = [Line2D([0], [0], color=graph['color'], linestyle=graph['linestyle'], linewidth=graph['legend_linewidth'],
                          label=graph['label']) for graph in plot4]
ax4.legend(handles=legend_elements, loc='lower right', handlelength=1.6, handleheight=1.8, labelspacing=0.05, ncol=2,
          columnspacing=1)
ax4.set_xlim([-0.02, 2.02])
ax4.set_yticks(np.arange(-45, 43, step=15))
ax4.set_ylim([-42,42])
ax4.vlines(touch_down_sample0*time_log[1]-time_log[start], ax4.get_ylim()[0], ax4.get_ylim()[1], linestyles='solid', zorder=10)
ax4.vlines(touch_down_sample1*time_log[1]-time_log[start], ax4.get_ylim()[0], ax4.get_ylim()[1], linestyles='solid', zorder=10)
ax4.vlines(lift_off_sample1*time_log[1]-time_log[start], ax4.get_ylim()[0], ax4.get_ylim()[1], linestyles='dashed', zorder=10)

fig.align_ylabels(fig.axes)
fig.tight_layout()



plt.show()





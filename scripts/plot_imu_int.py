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

directory = os.environ['LOCOSIM_DIR'] + '/landing_controller/simulations/20231204_160746/'
filename = "DATA.mat"
DATA = loadmat(directory + filename)
# Convert dictionary entries into local variables
# locals().update(DATA)

time_log = DATA['time_log'].flatten()
baseTwistW_legOdom_log = DATA['baseTwistW_legOdom_log']
baseLinTwistImuW_log = DATA['baseLinTwistImuW_log']
baseTwistW_des_log = DATA['baseTwistW_des_log']
del DATA

from base_controllers.utils.common_functions import *

sharex=True
sharey=False



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
mpl.rcParams['figure.figsize'] =  9.2, 10.2
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

###########################
# FIGURE GRFs and torques #
###########################
from  base_controllers.utils.common_functions import *
fig = plt.figure(4)

ax1 = plt.subplot2grid((3, 1), (0, 0), colspan=2)
plot1= []

#### desired pose
# lo x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = baseTwistW_legOdom_log[0, 6600:16500]
graph['label'] = r'Leg Odom'
graph['color'] = 'r'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)
# imu x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = baseLinTwistImuW_log[0, 6600:16500]
graph['label'] = r'IMU'
graph['color'] = 'b'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)
#des x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] =baseTwistW_des_log[0, 6600:16500]
graph['label'] = r'des'
graph['color'] = 'g'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot1.append(graph)

for graph in plot1:
    ax1.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])

legend_elements = [Line2D([0], [0], color=graph['color'], linestyle=graph['linestyle'], linewidth=graph['legend_linewidth'],
               label=graph['label']) for graph in plot1]
ax1.legend(handles=legend_elements, loc='upper right', handlelength=1.6, handleheight=1.8, labelspacing=0.05, ncol=1,
          columnspacing=1)


ax1.set_ylabel(r"$\hat{v}^{x}$ [m/s]")



ax2 = plt.subplot2grid((3, 1), (1, 0), colspan=2)
plot2= []
# lo x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = 0.2*baseTwistW_legOdom_log[1, 6600:16500]
graph['label'] = r'Leg Odom'
graph['color'] = 'r'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)
# imu x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = baseLinTwistImuW_log[1, 6600:16500]
graph['label'] = r'IMU'
graph['color'] = 'b'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)
#des x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] =baseTwistW_des_log[1, 6600:16500]
graph['label'] = r'des'
graph['color'] = 'g'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot2.append(graph)

for graph in plot2:
    ax2.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])


ax2.set_ylabel(r"$\hat{v}^{y}$ [m/s]")


ax3 = plt.subplot2grid((3, 1), (2, 0), colspan=2)
plot3= []
# lo x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = baseTwistW_legOdom_log[2, 6600:16500]
graph['label'] = r'Leg Odom'
graph['color'] = 'r'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)
# imu x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] = baseLinTwistImuW_log[2, 6600:16500]
graph['label'] = r'IMU'
graph['color'] = 'b'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)
#des x
graph = {}
graph['x'] = time_log[6600:16500] - time_log[6600]
graph['y'] =baseTwistW_des_log[2, 6600:16500]
graph['label'] = r'des'
graph['color'] = 'g'
graph['linestyle'] = '-'
graph['linewidth'] = 4
graph['legend_linewidth'] = 2
plot3.append(graph)

for graph in plot1:
    ax3.plot(graph['x'], graph['y'] , linestyle=graph['linestyle'],
             color=graph['color'], linewidth=graph['linewidth'])


ax3.set_xlabel(r"time [s]")
ax3.set_ylabel(r"$\hat{v}^{x}$ [m/s]")

fig.align_ylabels(fig.axes)
fig.tight_layout()





plt.show()





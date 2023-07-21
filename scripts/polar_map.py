import sys
sys.path.append('../')
from landing_controller.controller.utility import *

np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)
import os

import matplotlib as mpl
size_font = 16
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
mpl.rcParams['legend.fontsize'] = size_font - 2
mpl.rcParams['legend.title_fontsize'] = size_font - 2
mpl.rcParams['legend.loc'] = 'best'
mpl.rcParams['figure.facecolor'] = 'white'
mpl.rcParams['figure.figsize'] = 5, 4
mpl.rcParams['savefig.format'] = 'pdf'
# mpl.rcParams['mathtext.fontset'] = 'dejavuserif'
# mpl.rcParams['mathtext.bf'] = 'serif:bold'
plt.rcParams.update(
    {
        "text.usetex": True,
        "text.latex.preamble": r"\usepackage{bm}",
        "pgf.texsystem": "pdflatex",
        # Enforce default LaTeX font.
        # "font.family": "serif",
        "font.serif": ["Computer Modern"],
    }
)

filename = os.environ['LOCOSIM_DIR'] + '/landing_controller/simulations/inPaper/ResultsNoise/log.txt'
f = open(filename)

mags = []
phases_deg = []
success_rate = []

for line in f:
    if "Computing success" in line:
        mags_tmp = []
        phases_deg_tmp = []
        success_rate_tmp = []

        mags.append(mags_tmp)
        phases_deg.append(phases_deg_tmp)
        success_rate.append(success_rate_tmp)

    if "--> Tested magnitude" in line:
        start_mag = line.find(": ") + len(": ")
        end_mag = line.find(" [m/s]")
        mag = float(line[start_mag:end_mag])
        mags_tmp.append(mag)

        start_phase = line.find("phase ") + len("phase ")
        end_phase = line.find(" deg")
        phase_deg = float(line[start_phase:end_phase])
        phases_deg_tmp.append(phase_deg)

        start_succ = line.find("rate: ") + len("rate: ")
        end_succ = -1
        succ = float(line[start_succ:end_succ])
        success_rate_tmp.append(succ)


mags = np.array(mags)
phases_deg = np.array(phases_deg)
success_rate = np.array(success_rate)


from matplotlib.ticker import MaxNLocator
from matplotlib.colors import BoundaryNorm
levels = MaxNLocator().tick_values(0,1)

cmax = 0.8
cmin = 0.05
cmap = plt.cm.get_cmap('Blues')#.reversed()# check for the list plt.colormaps()
cmap = mpl.colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=cmin, b=cmax),
        cmap(np.linspace(cmin, cmax, 10)))

# fig1, ax1 = plt.subplots(1, 1)
# norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
# #pcm = ax1.pcolormesh(phases_deg, mags, success_rate,  cmap='Blues')
# pcm = plt.pcolor(phases_deg, mags, success_rate, cmap=cmap)
# cbar = fig1.colorbar(pcm, ax=ax1)
# ax1.set_ylabel(r'magnitude [m/s]')
# ax1.set_xlabel(r'phase [deg]')
# cbar.ax.set_ylabel(r'success rate', rotation=90)
# ax1.set_xticks(phases_deg[:,0])
#
# fig1.tight_layout()
# plt.show()
#
#
# mpl.rcParams['figure.figsize'] = 6.5,5
# fig2, ax2 = plt.subplots(1, 1)
# cf = ax2.contourf(phases_deg + 30/2, mags + 0.1/2, success_rate, levels=levels, cmap=cmap)
# cbar = fig2.colorbar(cf, ax=ax2)
# ax2.set_ylabel(r'magnitude [m/s]')
# ax2.set_xlabel(r'phase [deg]')
# cbar.ax.set_ylabel(r'success rate', rotation=90)
# ax2.set_xticks(phases_deg[:,0])
#
# fig2.tight_layout()
# plt.show()
#
#
# fig3, ax3 = plt.subplots(subplot_kw={'projection': 'polar'})
# phases_deg1 = np.vstack([phases_deg,-phases_deg[0, :]])
# mags1 = np.vstack([mags, mags[0, :]])
# success_rate1 = np.vstack([success_rate, success_rate[0,:]])
# pcm = ax3.pcolormesh(phases_deg1 * np.pi/180, mags1, success_rate1, cmap=cmap)
# cbar = fig3.colorbar(pcm, ax=ax3)
# cbar.ax.get_yaxis().labelpad = 15
# cbar.ax.set_ylabel(r'success rate', rotation=90)
# step = np.abs(phases_deg[1, 0]-phases_deg[0,0])
# phase_rad =np.arange(0,360, step)*np.pi/180
# ax3.set_xticks(phase_rad)
# ax3.tick_params(axis='x', which='major')
# ax3.set_yticklabels(['0', '', '1', '', '2', '', '3', 'm/s'])
#
# ax3.set_rmax(3)
# rticks = np.arange(0, 4, 0.5)
# ax3.set_rticks(rticks)
#
# fig3.tight_layout()
# plt.show()

import matplotlib as mpl
mpl.use('pgf')
fig4, ax4 = plt.subplots(subplot_kw={'projection': 'polar'})
phases_deg1 = np.vstack([phases_deg,-phases_deg[0, :]])
mags1 = np.vstack([mags, mags[0, :]])
success_rate1 = np.vstack([success_rate, success_rate[0,:]])

cf = ax4.contourf( (phases_deg1+30/2) * np.pi/180, mags1+0.1/2 , success_rate1, levels=levels, cmap=cmap)
cbar = fig4.colorbar(cf, ax=ax4, pad = 0.14)
cbar.ax.get_yaxis().labelpad = 10
cbar.ax.set_ylabel(r'success rate', rotation=90)
step = np.abs(phases_deg[1, 0]-phases_deg[0,0])
phase_rad =np.arange(0,360, step)*np.pi/180
ax4.set_xticks(phase_rad)
ax4.tick_params(axis='x', which='major', pad=12)
ax4.set_yticklabels(['0', '', '1', '', '2', '', '3', 'm/s'])

ax4.set_rmax(3)
rticks = np.arange(0, 4, 0.5)
ax4.set_rticks(rticks)

fig4.tight_layout()
plt.show()
plt.savefig('success.pdf', backend='pgf')


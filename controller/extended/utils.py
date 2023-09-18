import numpy as np
from scipy.special import comb
from scipy.optimize import linprog
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
import matplotlib.colors as colors
import matplotlib.cm as cmx
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Polygon


def plotOptimalBezier(time, pos, vel, acc, Tsol, pmin, pmax, p0, pf, v0, amax, ctrl_points):
    fig, axs = plt.subplots(3, 1, figsize=(10, 5))
    fig.suptitle(str(Tsol))
    axs[0].plot(time, pos)
    axs[0].set_ylabel('position [m]')
    axs[0].hlines(pmin, 0, time[-1], linestyles='dashed')
    axs[0].hlines(pmax, 0, time[-1], linestyles='dashed')
    axs[0].plot(0, p0, marker="o", markersize=5)
    color0 = axs[0].lines[0].get_color()
    for tau in ctrl_points:
        axs[0].vlines(float(tau * Tsol), pmin, pmax, linestyles='dashed', colors=color0)
    if pf is not None and (pmin < pf < pmax):
        axs[0].plot(time[-1], pf, marker="o", markersize=5)
    axs[0].grid()
    
    axs[1].plot(time, vel)
    axs[1].set_ylabel('velocity [m/s]')
    axs[1].plot(0, v0, marker="o", markersize=5)
    for tau in ctrl_points:
        axs[1].vlines(float(tau * Tsol), vel[0], vel[-1], linestyles='dashed', colors=color0)
    axs[1].grid()
    
    axs[2].plot(time, acc)
    axs[2].set_xlabel('time [s]')
    axs[2].set_ylabel('acceleration [m/s^2]')
    axs[2].hlines(0, 0, time[-1], linestyles='dashed')
    axs[2].hlines(amax, 0, time[-1], linestyles='dashed')
    for tau in ctrl_points:
        axs[2].vlines(float(tau * Tsol), 0, amax, linestyles='dashed', colors=color0)
    axs[2].grid()
    
    plt.subplots_adjust(right=0.85)


def plot_ref(time, pos, vel, acc, zmp_xy = None, projected_zmp = None, title=None, t_star = None):
    fig, axs = plt.subplots(3, 1, figsize=(10, 5), sharex=True)
    if title is not None:
        fig.suptitle(title)

    axs[0].set_ylabel("$c$ [$m$]")

    axs[0].plot(time, pos.T)

    if zmp_xy is not None:
        axs[0].plot(time, np.ones_like(time) * zmp_xy[0])
        axs[0].text(-0.02 * time[-1], zmp_xy[0], '$u^x_{opt}$')
    if projected_zmp is not None:
        axs[0].plot(time, np.ones_like(time) * projected_zmp[0], color=axs[0].lines[-1].get_color(),
                    linestyle='--')
        axs[0].text(-0.02 * time[-1], projected_zmp[0], '$u^x_{pr}$')

    if zmp_xy is not None:
        axs[0].plot(time, np.ones_like(time) * zmp_xy[1])
        axs[0].text(-0.02 * time[-1], zmp_xy[1], '$u^y_{opt}$')
    if projected_zmp is not None:
        axs[0].plot(time, np.ones_like(time) * projected_zmp[1], color=axs[0].lines[-1].get_color(),
                    linestyle='--')
        axs[0].text(-0.02 * time[-1], projected_zmp[1], '$u^y_{pr}$')

    if t_star is not None:
        axs[0].vlines(t_star, pos.min(), pos.max())
        axs[1].vlines(t_star, vel.min(), vel.max())
        axs[2].vlines(t_star, acc.min(), acc.max())

    axs[0].grid()

    axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
    axs[1].plot(time, vel.T)
    axs[1].grid()

    axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
    axs[2].plot(time, acc.T)
    axs[2].grid()

    legend = [Line2D([0], [0], color=axs[0].lines[0].get_color(), lw=4, label="x"),
              Line2D([0], [0], color=axs[0].lines[1].get_color(), lw=4, label="y"),
              Line2D([0], [0], color=axs[0].lines[2].get_color(), lw=4, label="z")]

    fig.legend(handles=legend,
               loc="center right",  # Position of legend
               borderaxespad=1,  # Small spacing around legend box
               )

    plt.subplots_adjust(right=0.85)
    plt.draw()
    plt.show()
    return fig


def plot_3D(pos, zmp_xy, projected_zmp, feasibility, feasibility_zmp, ctrl_indexes, feet = None, title=None):
    fig = plt.figure()
    if title is not None:
        fig.suptitle(title)
    ax = fig.add_subplot(projection='3d')
    plt.plot(pos[0], pos[1], pos[2], label='$c$')

    for id in ctrl_indexes:
        if id == ctrl_indexes[-1]:
            id = -1
        ax.scatter([pos[0, id]], [pos[1, id]], [pos[2, id]], color='b',
                   marker='o', s=80, alpha=0.6)

    scale = np.linspace(50, 150, len(feasibility.region))
    jet = plt.get_cmap('RdBu')
    cNorm = colors.Normalize(vmin=55, vmax=145)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

    idx = 5

    for kin_reg in feasibility.region:
        colorVal = scalarMap.to_rgba(scale[idx])
        if kin_reg[0, 2] == feasibility_zmp.nearest_h:
            idx_h = idx
        idx -= 1

        p = Polygon(kin_reg[:, :2] + projected_zmp.reshape(1,2), facecolor=colorVal, fill=True, alpha=0.1)
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=kin_reg[0, 2], zdir="z")
        p = Polygon(kin_reg[:, :2] + projected_zmp.reshape(1,2), facecolor=colorVal, fill=False, edgecolor=colorVal,
                    alpha=0.8)
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=kin_reg[0, 2], zdir="z")

    ax.scatter([projected_zmp[0]], [projected_zmp[1]], [0.], color='b', marker='*',
               s=40, label='Pr$(u_{opt}, FR_{l_{0}})$')
    ax.scatter([zmp_xy[0]], [zmp_xy[1]], [0.], color='r', marker='o', s=80, label='$u_{opt}$', alpha=0.6)

    if feet is not None:
        ax.scatter([feet[:, 0] + projected_zmp[0]], [feet[:, 1] + projected_zmp[1]], [feet[:, 2]], color='k')


    colorVal = scalarMap.to_rgba(scale[0])
    p = Polygon(-feasibility_zmp.region[:, :2], edgecolor='k', facecolor=colorVal, label='$ FR_{l_{0}}$',
                alpha=0.1)
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=0., zdir="z")


    floorx_max = np.min( [np.nanmax([np.abs(zmp_xy[0]), np.abs(feasibility_zmp.region[:, 0]).max(),
                         np.abs(pos[0, :]).max()]), 0.5]) + 0.05
    floory_max =  np.min( [np.nanmax([np.abs(zmp_xy[1]), np.abs(feasibility_zmp.region[:, 0]).max(),
                         np.abs(pos[1, :]).max()]), 0.5]) + 0.05
    ax.set_xlim(-0.2, 0.6)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.0, feasibility_zmp.region[:, 2].max() + 0.05)
    ax.set_xlabel("$c^{x}$ [$m$]")
    ax.set_ylabel("$c^{y}$ [$m$]")
    ax.set_zlabel("$c^{z}$ [$m$]")

    plt.legend()
    plt.draw()
    plt.show()

    return fig

def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])

def bezier(w, tau):
    b = 0.
    deg = w.shape[0] - 1
    for k in range(0, deg + 1):
        b += comb(deg, k, exact=True) * w[k] * (1 - tau) ** (deg - k) * tau ** k
    return b

def bezierTraj(w, T0=0, Tf=1, step=0.002):
    T = np.arange(T0, Tf, step)
    deltaT = T[-1] - T[0]
    B = np.zeros_like(T)
    for i in range(T.shape[0]):
        tau = (T[i] - T[0]) / deltaT
        B[i] = bezier(w, tau)
    return T, B

# noinspection PyDeprecation
def projectPointToPolygon(A, b, point):
    # (A, b) describes a 2d polygon, i.e. the point x belongs to the polygon if A@x + b <= 0.
    # The origin is inside the polygon
    # The function finds the intersection between the line joining the origin and the given point and the polygon,
    # in the direction of the point, solving the following NLP
    # max k
    # s.t. x = k*[point_x, point_y]
    #      Ax + b <= 0
    #      k >= 0
    point = point.reshape(2, 1)
    c = [-1]
    A = A @ point
    b = b
    bounds = (0, None)
    res = linprog(c, A_ub=A.reshape(A.shape[0], 1), b_ub=-b, bounds=bounds, method='highs-ds')
    projected_point = res.x * point
    return projected_point.reshape(2,)

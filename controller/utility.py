import matplotlib.pyplot as plt
import numpy as np
from termcolor import colored
DEG2RAD = np.pi/180


def initCond2str(init_cond, speedUpDown=1.):
    ICstr = ''
    ICstr += 'id: '   + init_cond['id'] + '\n'
    ICstr += 'name: ' + init_cond['name'] + '\n'
    ICstr += 'base height: ' + str(np.round(init_cond['pose'][2], 3)) + ' m\n'
    ICstr += 'base pitch: ' + str(np.round(init_cond['pose'][4], 3)/DEG2RAD) + ' deg\n'
    ICstr += 'base fw vel: ' + str(np.round(init_cond['twist'][0], 3)) + ' m/s\n'
    ICstr += 'base lat vel: ' + str(np.round(init_cond['twist'][1], 3)) + ' m/s\n'
    ICstr += 'base pitch vel: ' + str(np.round(init_cond['twist'][4], 3)/DEG2RAD) + ' deg/s\n'
    ICstr += 't_video: ' + str(init_cond['t_video']) + '\n'
    if speedUpDown > 0. and speedUpDown != 1.:
        ICstr += 't_video_speedUpDown: ' + str(init_cond['t_video']/speedUpDown) + '\n'
    return ICstr



def manipulateFig(fig, filename, PLOT_SETTINGS):
    fig.set_size_inches([PLOT_SETTINGS['width_inches'], PLOT_SETTINGS['height_inches']])

    if PLOT_SETTINGS['save']:
        plt.savefig(PLOT_SETTINGS['directory_path'] + '/' + filename+ '.png')
        plt.close()
        print(colored('Plot ' + PLOT_SETTINGS['directory_path'] + '/' + filename+ '.png saved', color='green'), flush=True)

    if PLOT_SETTINGS['show']:
        plt.show()
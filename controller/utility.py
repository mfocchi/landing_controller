import matplotlib.pyplot as plt
import numpy as np
from base_controllers.utils.common_functions import *
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


def manipulateFig(fig, filename, PLOT_SETTINGS, verbose = False):
    fig.set_size_inches([PLOT_SETTINGS['width_inches'], PLOT_SETTINGS['height_inches']])

    if PLOT_SETTINGS['save']:
        plt.savefig(PLOT_SETTINGS['directory_path'] + '/' + filename+ '.png')
        plt.close()
        del fig
        if verbose:
            print('Plot ' + filename+ '.png saved', flush=True)

    if PLOT_SETTINGS['show']:
        plt.show()


def makePlots(p, figures, PLOT_SETTINGS, verbose = False):
    id = 0
    for name in figures:
        if name == 'q' and figures[name]:
            # joint position
            fig = plotJoint('position', id, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log)
            manipulateFig(fig, 'q', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'qd' and figures[name]:
            # joint velocity
            fig = plotJoint('velocity', id, time_log=p.time_log.flatten(), qd_log=p.qd_log,
                            qd_des_log=p.qd_des_log)
            manipulateFig(fig, 'qd', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'tau' and figures[name]:
            # joint torques
            fig = plotJoint('torque', id, time_log=p.time_log.flatten(), tau_ffwd_log=p.tau_ffwd_log,
                            tau_log=p.tau_log,
                            tau_des_log=p.tau_des_log)
            manipulateFig(fig, 'tau', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'com' and figures[name]:
            # com position
            fig = plotCoM('position', 1, time_log=p.time_log.flatten(), basePoseW=p.comPoseW_log,
                          des_basePoseW=p.comPoseW_des_log)
            manipulateFig(fig, 'com', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'vcom' and figures[name]:
            # com velocity
            fig = plotCoM('velocity', id, time_log=p.time_log.flatten(), baseTwistW=p.comTwistW_log,
                          des_baseTwistW=p.comTwistW_des_log)
            manipulateFig(fig, 'vcom', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'w_contacts' and figures[name]:
            # feet position in w-frame and contact flag
            fig = plotFeet(id, time_log=p.time_log.flatten(), des_feet=p.W_contacts_des_log,
                           act_feet=p.W_contacts_log, contact_states=p.contact_state_log)
            manipulateFig(fig, 'W_feet', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'b_contacts' and figures[name]:
            # feet position in b-frame and contact flag
            fig = plotFeet(id, time_log=p.time_log.flatten(), des_feet=p.B_contacts_des_log,
                           act_feet=p.B_contacts_log, contact_states=p.contact_state_log)
            manipulateFig(fig, 'B_feet', PLOT_SETTINGS, verbose = False)
            id += 1

        elif name == 'grfs_contacts' and figures[name]:
            # force in world
            fig = plotGRFs_withContacts(id, time_log=p.time_log.flatten(), des_forces=p.grForcesW_des_log,
                                        act_forces=p.grForcesW_log, contact_states=p.contact_state_log)
            manipulateFig(fig, 'grfs', PLOT_SETTINGS, verbose = False)
            id += 1

        else:
            print('Cannot understand ' + name)

def saveWorkspace(path):
    filename = path + '/workspace.out'
    my_shelf = shelve.open(filename, 'n')  # 'n' for new

    for key in dir():
        try:
            my_shelf[key] = globals()[key]
        except TypeError:
            # __builtins__, my_shelf, and imported modules can not be shelved.
            print('ERROR shelving: {0}'.format(key))
    my_shelf.close()


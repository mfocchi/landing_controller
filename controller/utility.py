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


            # joint position
            fig = plotJoint('position', 0, time_log=self.p.time_log.flatten(), q_log=self.p.q_log, q_des_log=self.p.q_des_log)
            manipulateFig(fig, 'q', PLOT_SETTINGS)

            # joint torques
            fig = plotJoint('torque', fig.number + 1, time_log=self.p.time_log.flatten(), tau_ffwd_log=self.p.tau_ffwd_log,
                            tau_log=self.p.tau_log,
                            tau_des_log=self.p.tau_des_log)
            manipulateFig(fig, 'tau', PLOT_SETTINGS)

            # com position
            # self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:] += self.p.comPoseW_log[
            #     0, self.lc.jumping_data_times.touch_down.sample]
            # self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:] += self.p.comPoseW_log[
            #     1, self.lc.jumping_data_times.touch_down.sample]

            fig = plotCoM('position', 1, time_log=self.p.time_log.flatten(), basePoseW=self.p.comPoseW_log,
                          des_basePoseW=self.p.comPoseW_des_log)
            manipulateFig(fig, 'com', PLOT_SETTINGS)

            # com velocity
            fig = plotCoM('velocity', fig.number + 1, time_log=self.p.time_log.flatten(), baseTwistW=self.p.comTwistW_log,
                          des_baseTwistW=self.p.comTwistW_des_log)
            manipulateFig(fig, 'vcom', PLOT_SETTINGS)

            # # feet position in w-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=self.p.time_log.flatten(), des_feet=self.p.W_contacts_des_log,
            #                act_feet=self.p.W_contacts_log, contact_states=self.p.contact_state_log)
            # manipulateFig(fig, 'W_feet', PLOT_SETTINGS)
            #
            # # feet position in b-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=self.p.time_log.flatten(), des_feet=self.p.B_contacts_des_log,
            #                act_feet=self.p.B_contacts_log, contact_states=self.p.contact_state_log)
            # manipulateFig(fig, 'B_feet', PLOT_SETTINGS)

            # force in world
            fig = plotGRFs_withContacts(fig.number + 1, time_log=self.p.time_log.flatten(), des_forces=self.p.grForcesW_gt_log,
                                        act_forces=self.p.grForcesW_des_log, contact_states=self.p.contact_state_log)
            manipulateFig(fig, 'grfs', PLOT_SETTINGS)

            # margins
            # fig = plt.figure(fig.number + 1)
            # ax = fig.subplots()
            # X_vals = []
            # Y_vals = []
            # T_contacts = self.p.W_contacts.copy()
            # for c in T_contacts:
            #     c[0] -= self.p.comPoseW_log[0, self.lc.jumping_data_times.touch_down.sample]
            #     c[1] -= self.p.comPoseW_log[1, self.lc.jumping_data_times.touch_down.sample]
            # T_sp = self.p.support_poly(T_contacts)
            # for side in T_sp:
            #     X_vals.append(T_sp[side]['p0'][0])
            #     Y_vals.append(T_sp[side]['p0'][1])
            # X_vals.append(X_vals[0])
            # Y_vals.append(Y_vals[0])
            # plt.plot(X_vals, Y_vals, lw=6)
            #
            # eta, limit_zmp, marg_vx, marg_vy, limit_vx, limit_vy = self.lc.velocity_margin(T_sp)
            # # for plotting in world, we should add the position as td
            # ax.add_patch(plt.Circle((self.lc.slip_dyn.zmp_xy[0], self.lc.slip_dyn.zmp_xy[1]), 0.005, color='r'))
            # ax.add_patch(plt.Circle((limit_zmp[0], limit_zmp[1]), 0.005, color='b'))
            # n = np.linalg.norm(self.lc.init_vel)
            #
            # plt.plot([0, limit_vx / n], [0, limit_vy / n], lw=4)
            # plt.plot([0, self.lc.init_vel[0] / n], [0, self.lc.init_vel[1] / n], lw=6)
            #
            # # self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:] -= self.p.comPoseW_log[ 0, self.lc.jumping_data_times.touch_down.sample]
            # # self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:] -= self.p.comPoseW_log[1, self.lc.jumping_data_times.touch_down.sample]
            # # plt.plot(self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:], self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:])
            # ax.set_aspect('equal', adjustable='box')
            #
            # plt.legend(
            #     ["support polygon", "zmp", "limit zmp", "limit TD velocity 'normalized'", "TD velocity normalized"],
            #     fontsize=20)
            #
            # manipulateFig(fig, 'margins', PLOT_SETTINGS)
            #
            # # init cond file
            # if PLOT_SETTINGS['save']:
            #     f = open(PLOT_SETTINGS['directory_path'] + "/simulation.txt", "w")
            #
            #     f.write(initCond2str(simulation, PLOT_SETTINGS['speedUpDown']))
            #     f.close()
            #     print(colored('File ' + PLOT_SETTINGS['directory_path'] + '/simulation.txt saved', color='green'),
            #           flush=True)

        ######################
        # success conditions #
        ######################
        # com close to des com
        if np.linalg.norm(self.p.comPoseW_des[0:3]-self.p.comPoseW[0:3]) > 0.1 :
            return 1

        # feet don't move too much
        for i in range(4):
            for c in self.p.W_contacts_log[0:3, self.lc.jumping_data_times.touch_down.sample:self.p.log_counter].T:
                if np.linalg.norm(c - self.p.W_contacts_TD[0]) > 0.03:
                    return 2
        return 0
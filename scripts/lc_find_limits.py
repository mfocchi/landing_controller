import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import rospy as ros
import sys
import os
from base_controllers.quadruped_controller import Controller
from controller.landing_manager import LandingManager
from controller import SETTINGS
from controller.utility import *


np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)


ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
use_gui = False

phase_deg_list = np.arange(-180, 180, 30)
magnitude_init_list = [2.8, 2.9, 2.4, 2., 1.9, 1.8, 1.7, 1.8, 1.9, 2., 2.4, 2.9] # these values are close to the limits for h = 0.7 m

if __name__ == '__main__':
    setSavePath(SETTINGS, 'simulations')

    if SETTINGS['save_log']:
        stdout = sys.stdout
        logfile = open(SETTINGS['save_path'] + "/log.txt", "w")

    try:
        p = Controller(ROBOT_NAME)
        if SETTINGS['VIDEO']['save']:
            world_name = 'camera_'+world_name

        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=False,
                          additional_args=['gui:=true',
                                           'go0_conf:=standDown',
                                           'pid_discrete_implementation:=false'])

        p.startupProcedure()

        if SETTINGS['VIDEO']['save']:
            init_video_frame = p.get_current_frame_file()

        lm = LandingManager(p, SETTINGS)
        sim_counter = 0
        if SETTINGS['save_log']:
            sys.stdout = logfile

        for simulation in SETTINGS['SIMS']:
            if simulation['name']!='find_limits':
                sys.stdout = stdout
                assert simulation['name']!='find_limits', "You should use lc_simulation.py. Exit..."
        else:
            sim_str = findLimitsInitCond2str(simulation)
            print(sim_str)
            magnitude_list = []
            for ii, phase_deg in enumerate(phase_deg_list):
                if SETTINGS['verbose']:
                    print('Simulation ' + simulation_try['id'] + " - Searching maximum magnitude for phase: " + str(
                        phase_deg) + " deg", flush=True)
                phase_rad = phase_deg_list * np.pi/180
                unit_baseTwist = np.array([np.cos(phase), np.sin(phase), 0., 0., 0., 0.])
                magnitude_try = simulation['magnitude_init_list'][ii]
                magnitude = 0.

                increase_mag = True
                succeed_once = False

                while True:
                    simulation_try = {'name': simulation['name'],
                                      'pose': simulation['pose'],
                                      'twist':  magnitude_try * unit_baseTwist,
                                      'useWBC': simulation['useWBC'],
                                      'useIK': simulation['useIK'],
                                      'typeWBC': simulation['typeWBC'],  # or 'qp' (used only if useWBC is True)
                                      'id': '',
                                      't_video': 0.0,
                                      'directory': ''}
                    setId(simulation_try, sim_counter)
                    if SETTINGS['verbose']:
                        print("--> Testing magnitude: " + str(magnitude_try) + " [m/s]", flush=True)

                    ret = s.succeed(simulation_try['id'],
                                    simulation_try['pose'],
                                    simulation_try['twist'],
                                    simulation_try['useIK'],
                                    simulation_try['useWBC'],
                                    simulation_try['typeWBC'],
                                    simplified=False)

                    sim_counter += 1

                    if ret == 0:
                        print('    Succeed')
                        if inc_mag:  # simulation succeed, increase the init vel and restart
                            succ_once = True
                            magnitude = magnitude_try
                            magnitude_try = np.around(magnitude_try + 0.1, 1)
                        else:  # simulation succeed, init vel has been only decreased -> solution found
                            succ_once = True
                            magnitude = magnitude_try
                            break
                    else:
                        print('    Failed')
                        if ret == 1:
                            print('    Fault cause: cannot track com', flush=True)
                        elif ret == 2:
                            print('    Fault cause: A foot moved', flush=True)
                        if not succ_once:  # simulation failed and never succeeded before, decrease inc_mag
                            inc_mag = False
                            magnitude_try = np.around(magnitude_try - 0.1, 1)
                            if magnitude_try < 0.:
                                print('    A solution does not exist', flush=True)
                                break
                        else:  # simulation succeed, init vel has been only increased -> solution found
                            break

                magnitude_list.append(magnitude)
                print("\nLimit magnitude for " + str(phase_deg) + " deg: " + str(magnitude) + " [m/s]")
                print('-' * 60)

            print('RESULTS for height', np.around(simulation['pose'][2], 2), '[m] with controller', ctrl)
            for ii in range(len(phase_deg_list)):
                print('phase:', phase_deg_list[ii], 'deg, limit magnitude:', magnitude_list[ii])

            n_phases = len(phase_deg_list)
            print('phase (deg):' + ' ' * 11, end='')
            for i in range(n_phases):
                if i != n_phases - 1:
                    print(str(phase_deg_list[i]) + ', ', end='')
                else:
                    print(str(phase_deg_list[i]))

            print('limit magnitude (m/s): ', end='')
            for i in range(n_phases):
                if i != 5:
                    print(str(magnitude_list[i]) + ', ', end='')
                else:
                    print(str(magnitude_list[i]))


    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        print(e)
    finally:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        # save all for later analysis
        if SETTINGS['WORKSPACE']['save']:
            saveWorkspace(SETTINGS['save_path'])

        # store all the init conds
        if SETTINGS['INIT_CONDS']['save_all']:
            saveAllInitConds(SETTINGS['save_path'], SETTINGS['SIMS'], SETTINGS['VIDEO']['speedUpDown'], SETTINGS['verbose'])


        if SETTINGS['VIDEO']['save']:
            # save the video
            p.save_video(SETTINGS['save_path'], start_file=init_video_frame, speedUpDown=SETTINGS['VIDEO']['speedUpDown'])
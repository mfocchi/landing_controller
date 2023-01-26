import matplotlib
matplotlib.use('TkAgg')
from base_controllers.controller_new import Controller
from controller.settings import SETTINGS
from controller.utility import initCond2str, Simulation
from base_controllers.utils.common_functions import * #(matplotlib.pyplot, numpy, plots, ros)
import os
from termcolor import colored
import shelve


np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed numer of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)

ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
phase_deg_list = np.arange(-180, 180, 30)
magnitude_sim_list = []

# these values are close to the limits for h = 0.7 m
magnitude_init_list = [2.8, 2.9, 2.4, 2., 1.9, 1.8, 1.7, 1.8, 1.9, 2., 2.4, 2.9]


if __name__ == '__main__':
    stdout = sys.stdout
    logfile = open(SETTINGS['PLOTS']['save_path'] + "/log.txt", "w")


    p = Controller(ROBOT_NAME)
    try:

        if SETTINGS['PLOTS']['video']:
            world_name = 'camera_'+world_name

        p.startController(world_name=world_name,
                          additional_args=['gui:=False', 'go0_conf:=standDown'])

        p.startupProcedure()  # overloaded method

        if SETTINGS['PLOTS']['video']:
            init_video_frame = p.get_current_frame_file()

        if not SETTINGS['PLOTS']['show']:
            plt.ioff()


        s = Simulation(p)
        sim_counter = -1
        sys.stdout = logfile
        for simulation in SETTINGS['SIMS']:
            if simulation['name']!='find_limits':
                sim_counter += 1
                if sim_counter < 10:
                    simulation['id'] = '0' + str(sim_counter)
                else:
                    simulation['id'] = str(sim_counter)

                    t_start_video = p.time.copy()

                    simulation['t_video'] = t_start_video
                    if simulation['id'] != '00':
                        simulation['t_video'] += SETTINGS['SIMS'][sim_counter - 1]['t_video']

                print(colored("Starting simulation " + simulation['id'] + ': ' + simulation['name'], "blue"))
                s.succeed(simulation['id'],
                          simulation['pose'],
                          simulation['twist'],
                          simulation['useIK'],
                          simulation['useWBC'],
                          simulation['typeWBC'],
                          simplified=True)


            else:
                for ctrl in ['useWBC']:
                    print_str = '* Test height: ' + str(np.around(simulation['pose'][2], 2)) + ' [m], controller = ' + ctrl + ' *'
                    n_stars = len(print_str)
                    print('*' * n_stars)
                    print(print_str)
                    print('*' * n_stars)
                    magnitude_list = []
                    for ii, phase_deg in enumerate(phase_deg_list):

                        sim_counter += 1
                        if sim_counter < 10:
                            simulation['id'] = '0' + str(sim_counter)
                        else:
                            simulation['id'] = str(sim_counter)

                        phase = phase_deg * np.pi/180
                        unit_baseTwist = np.array([np.cos(phase), np.sin(phase), 0., 0., 0., 0.])
                        magnitude_try = simulation['magnitude_init_list'][ii]
                        magnitude = 0.

                        inc_mag = True
                        succ_once = False

                        while True:
                            simulation_try = {'name': simulation['name'],
                                              'pose': simulation['pose'],
                                              'twist': magnitude_try * unit_baseTwist,
                                              'useIK': ctrl == 'useIK',
                                              'useWBC': ctrl == 'useWBC',
                                              'typeWBC': simulation['typeWBC'],  #'projection' or 'qp' (used only if useWBC is True)
                                              'id': '',
                                              't_video': 0.0}
                            if sim_counter < 10:
                                simulation_try['id'] = '0' + str(sim_counter)
                            else:
                                simulation_try['id'] = str(sim_counter)


                            print('sim'+simulation_try['id']+" Searching maximum magnitude for a touch down velocity having phase: " + str(phase_deg) + " deg", flush=True)
                            print("--> Testing magnitude: " + str(magnitude_try) + " [m/s]", flush=True)

                            ret = s.succeed(simulation_try['id'],
                                            simulation_try['pose'],
                                            simulation_try['twist'],
                                            simulation_try['useIK'],
                                            simulation_try['useWBC'],
                                            simulation_try['typeWBC'],
                                            simplified=False)

                            sim_counter += 1

                            if ret==0:
                                print('    Succeed')
                                if inc_mag:            # simulation succeed, increase the init vel and restart
                                    succ_once = True
                                    magnitude = magnitude_try
                                    magnitude_try = np.around(magnitude_try+0.1, 1)
                                else:        # simulation succeed, init vel has been only decreased -> solution found
                                    succ_once = True
                                    magnitude = magnitude_try
                                    break
                            else:
                                print('    Failed')
                                if ret == 1:
                                    print('    Fault cause: cannot track com', flush=True)
                                elif ret == 2:
                                    print('    Fault cause: A foot moved', flush = True)
                                if not succ_once:  # simulation failed and never succeeded before, decrease inc_mag
                                    inc_mag = False
                                    magnitude_try = np.around(magnitude_try-0.1, 1)
                                    if magnitude_try < 0.:
                                        print('    A solution does not exist', flush=True)
                                        break
                                else:      # simulation succeed, init vel has been only increased -> solution found
                                    break


                        magnitude_list.append(magnitude)
                        print("\nLimit magnitude for " + str(phase_deg) + " deg: " + str(magnitude) + " [m/s]")
                        print('-'*60)

                    print('RESULTS for height',np.around(simulation['pose'][2], 2), '[m] with controller', ctrl)
                    for ii in range(len(phase_deg_list)):
                        print('phase:', phase_deg_list[ii], 'deg, limit magnitude:', magnitude_list[ii])

                    n_phases = len(phase_deg_list)
                    print('phase (deg):'+ ' '*11, end='')
                    for i in range(n_phases):
                        if i != n_phases-1:
                            print(str(phase_deg_list[i]) + ', ', end='')
                        else:
                            print(str(phase_deg_list[i]))

                    print('limit magnitude (m/s): ', end='')
                    for i in range(n_phases):
                        if i != 5:
                            print(str(magnitude_list[i]) + ', ', end='')
                        else:
                            print(str(magnitude_list[i]))


                    magnitude_sim_list.append(magnitude_list)





    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        logfile.close()
        sys.stdout = stdout
        print(e)
    finally:
        logfile.close()
        sys.stdout = stdout
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        # save all for later analysis
        if SETTINGS['WORKSPACE']['save']:

            filename = SETTINGS['PLOTS']['save_path'] + '/workspace.out'
            my_shelf = shelve.open(filename, 'n')  # 'n' for new

            for key in dir():
                try:
                    my_shelf[key] = globals()[key]
                except TypeError:
                    #
                    # __builtins__, my_shelf, and imported modules can not be shelved.
                    #
                    print('ERROR shelving: {0}'.format(key))
            my_shelf.close()


        # # store all the init conds
        # if SETTINGS['PLOTS']['save']:
        #     f = open(SETTINGS['PLOTS']['save_path'] + "/ALL_simulations.txt", "w")
        #     for simulation in SETTINGS['SIMS']:
        #         f.write(initCond2str(simulation, SETTINGS['PLOTS']['speedUpDown']) + '\n' + '-' * 10 + '\n')
        #     f.close()
        #     print(SETTINGS['PLOTS']['save_path'] + '/ALL_simulations.txt saved')

        if SETTINGS['PLOTS']['video']:
            # save the video
            p.save_video(SETTINGS['PLOTS']['save_path'], start_file=init_video_frame, speedUpDown=SETTINGS['PLOTS']['speedUpDown'])



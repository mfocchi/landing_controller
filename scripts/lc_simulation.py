import matplotlib
matplotlib.use('TkAgg')
import sys
sys.path.append('../')
from base_controllers.quadruped_controller import Controller
from landing_controller.controller.landing_manager import LandingManager
from landing_controller.controller import SETTINGS
from landing_controller.controller.utility import *


np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)


ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
use_gui = True


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

        lm = LandingManager(p, SETTINGS)
        sim_counter = -1
        if SETTINGS['save_log']:
            sys.stdout = logfile

        for simulation in SETTINGS['SIMS']:
            if simulation['name']=='find_limits':
                sys.stdout = stdout
                assert simulation['name']=='find_limits', "You should use lc_find_limits.py. Exit..."

            else:
                sim_counter += 1
                setId(simulation, sim_counter)

                if SETTINGS['verbose']:
                    print("Starting simulation " + simulation['id'] + ': ' + simulation['name'])
                ret = lm.run(sim_counter,
                       simulation['pose'],
                       simulation['twist'],
                       simulation['useIK'],
                       simulation['useWBC'],
                       simulation['typeWBC'],
                       simplified=False)
                if SETTINGS['verbose']:
                    print("--> simulation " + simulation['id'] + 'completed. Stabilized?' + str(ret))



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



        # store all the init conds
        if SETTINGS['INIT_CONDS']['save_all']:
            saveAllInitConds(SETTINGS['save_path'], SETTINGS['SIMS'], SETTINGS['VIDEO']['speedUpDown'], SETTINGS['verbose'])

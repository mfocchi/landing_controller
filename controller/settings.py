import matplotlib.pyplot as plt
import numpy as np
from screeninfo import get_monitors
import os
import datetime

SETTINGS = {}
#################
# plot settings #
#################
WORKSPACE_SETTINGS={}
WORKSPACE_SETTINGS['save'] = False
SETTINGS['WORKSPACE'] = WORKSPACE_SETTINGS

#################
# plot settings #
#################
PLOT_SETTINGS = {}
PLOT_SETTINGS['show'] = True
PLOT_SETTINGS['save'] = False
PLOT_SETTINGS['video'] = False
PLOT_SETTINGS['speedUpDown'] = 0.2

# do not modify the following
plt.ioff()
width_inches = 0.
height_inches = 0.
mm2inches = 0.0393701
for m in get_monitors():
    if m.is_primary:
        width_inches = m.width_mm * mm2inches
        height_inches = m.height_mm * mm2inches
        break
PLOT_SETTINGS['width_inches'] = width_inches
PLOT_SETTINGS['height_inches'] = height_inches
PLOT_SETTINGS['save_path'] = ''

now = datetime.datetime.now()
now_s = str(now)
now_s = now_s.replace('-', '')
now_s = now_s.replace(' ', '_')
now_s = now_s.replace(':', '')
now_s = now_s[: now_s.find('.')]
PLOT_SETTINGS['save_path'] = os.environ['LC_DIR'] + '/simulations/' + now_s
os.mkdir(PLOT_SETTINGS['save_path'])

PLOT_SETTINGS['directory_path'] = '' # filled at runtime if PLOT_SETTINGS['save']  is true

SETTINGS['PLOTS'] = PLOT_SETTINGS
#######################
# Simulation Settings #
#######################
# SIMS_SETTINGS is a list of dictionary containing the following fields:
# 'name' (str):             name of the jump, used for generate folders containing plots
# 'pose' (np.ndarray(6,)):  initial pose of the robot base
# 'twist'(np.ndarray(6,)):  initial twist of the robot base
# 'useWBC' (bool):          use wbc in landing phase (at least one between useWBC and useIK must be true)
# 'useIK'(bool):            use ik in landing phase
# 'typeWBC' (str):          'projection' or 'qp' (used only if useWBC is True)
# 'id' (str):               numeric identifier of the simulation, will be filled at run-time
# 't_video' (float):        start time in the video, will be filled at run-time

DEG2RAD = np.pi / 180
SIMS_SETTINGS = []
SIMS_SETTINGS.append({'name': 'A_high_jump',
                      'pose': np.array([0., 0., .6, 0., 0., 0.]),
                      'twist': np.array([1.0, 0., 0., 0., 0., 0.]),
                      'useWBC': True,
                      'useIK': False,
                      'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
                      'id': '',
                      't_video': 0.0})

# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., -20.*DEG2RAD, 0.]),
#                       'twist': np.array([1., 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})

# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0., 1.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0., -1.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., 1., 0., 10.*DEG2RAD, 0.]),
#                       'twist': np.array([1., 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., 1., 0., 0, 0.]),
#                       'twist': np.array([-1., 0., 0., 0., 0.5, 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., 0.85, 0., 0., 0.]),
#                       'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'B_lateral_mix',
#                       'pose': np.array([0., 0., 0.55, 0., 0., 0.]),
#                       'twist': np.array([0.3, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'B_lateral_mix',
#                       'pose': np.array([0., 0., 0.85, 0., 0., 0.]),
#                       'twist': np.array([0.3, 0.6, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'C_pitch',
#                       'pose': np.array([0., 0., 0.85, 0., -10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'C_pitch',
#                       'pose': np.array([0., 0., 0.85, 0., +10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'D_pitch_mix',
#                       'pose': np.array([0., 0., 0.85, 0., -10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'D_pitch_mix',
#                       'pose': np.array([0., 0., 0.85, 0., +10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'E_omega',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0., 0., 0., 0., 1., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'E_omega',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0.5, 0., 0., 0., 1., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})

# SIMS_SETTINGS.append({'name': 'find_limits',
#                       'pose': np.array([0., 0., 0.5, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list':  np.array([1.5, 1.4, 1.7, 2.2, 1.9, 2.2, 2.8, 2.2, 1.9, 2.1, 1.7, 1.4]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
#
# SIMS_SETTINGS.append({'name': 'find_limits',
#                       'pose': np.array([0., 0., .7, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list': np.array([3.0, 3.3, 2.7, 2.3, 2.1, 2.1, 2.2, 2.1, 2.0, 2.3, 2.7, 3.3]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
# SIMS_SETTINGS.append({'name': 'find_limits',
#                       'pose': np.array([0., 0., 1.2, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list': np.array([2.6, 2.2, 1.6, 1.3, 1.7, 1.3, 2.2, 1.1, 1.7, 1.4, 1.6, 2.1]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})
# SIMS_SETTINGS.append({'name': 'find_limits',
#                       'pose': np.array([0., 0., 2.0, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list': np.array([.80]*12),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       't_video': 0.0})

SETTINGS['SIMS'] = SIMS_SETTINGS

import os
import datetime
from controller.settings import *


SETTINGS = {}
SETTINGS['WORKSPACE'] = WORKSPACE_SETTINGS
SETTINGS['FIGURES'] = FIG_LIST
SETTINGS['PLOTS'] = PLOT_SETTINGS
SETTINGS['SIMS'] = SIMS_SETTINGS
SETTINGS['VIDEO'] = VIDEO
SETTINGS['INIT_CONDS'] = INIT_CONDS

SETTINGS['verbose'] = True
SETTINGS['save_log'] = False
SETTINGS['save_path'] = ''

if SETTINGS['WORKSPACE']['save'] or \
   SETTINGS['PLOTS']['save'] or \
   SETTINGS['save_log'] or \
   SETTINGS['INIT_CONDS']['save_all']:
    now = datetime.datetime.now()
    now_s = str(now)
    now_s = now_s.replace('-', '')
    now_s = now_s.replace(' ', '_')
    now_s = now_s.replace(':', '')
    now_s = now_s[: now_s.find('.')]
    SETTINGS['save_path'] = os.environ['LC_DIR'] + '/simulations/' + now_s
    os.mkdir(SETTINGS['save_path'])
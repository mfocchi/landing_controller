import matplotlib
matplotlib.use('TkAgg')
from base_controllers.utils.common_functions import *

phase_deg = np.array([-180, -150, -120, -90,   -60,  -30,  0,     30,   60,   90,  120,  150])

mag_h050 = np.array([  1.2,  1.4,  1.7,  2.8,  2.8,  2.9,  2.6,  2.4,  2.6,  2.7,  1.6,  1.3])
mag_h070 = np.array([  2.2,  2.2,  2.7,  2.9,  3.1,  2.7,  2.5,  2.8,  2.6,  3.0,  2.6,  1.9])
mag_h120 = np.array([  3.4,  3.1,  1.8,  3.2,  1.6,  1.3,  1.7,  1.1,  1.7,  1.5,  1.7,  2.9])

fig, ax = polar_char('Limits', 0, phase_deg, mag_h050, mag_h070, mag_h120)
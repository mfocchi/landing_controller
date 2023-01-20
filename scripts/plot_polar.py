import matplotlib
matplotlib.use('TkAgg')
from base_controllers.utils.common_functions import *

phase_deg = np.arange(-180, 180, 30)

mag_h050 = np.array([1.5, 1.4, 1.7, 2.2, 1.9, 2.2, 2.8, 2.2, 1.9, 2.1, 1.7, 1.4])
mag_h070 = np.array([3.0, 3.3, 2.7, 2.3, 2.1, 2.1, 2.2, 2.1, 2.0, 2.3, 2.7, 3.3])
mag_h120 = np.array([2.6, 2.2, 1.6, 1.3, 1.7, 1.3, 2.2, 1.1, 1.7, 1.4, 1.6, 2.1])

fig, ax = polar_char('h 50 cm', 0, phase_deg, mag_h050, mag_h070, mag_h120)
import matplotlib
matplotlib.use('TkAgg')
import sys
sys.path.append('../')
from landing_controller.controller.utility import *

np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)

from scipy.io import loadmat

filename = ''

DATA = loadmat(filename)

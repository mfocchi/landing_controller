from base_controllers.utils.common_functions import *

phase_deg = np.array([-180, -150, -120, -90,   -60,  -30,  0,     30,   60,   90,  120,  150])

# using pseudo-inverse to map wrences into grfs
mag_h035_lc = np.array([  1.8,  1.9,  2.8,  2.2,  1.8,  1.5,  1.2,  1.5,  1.7,  2.4,  3.0,  2.0])
mag_h050_lc = np.array([  2.4,  2.8,  2.3,  2.3,  1.9,  1.8,  1.8,  1.7,  1.8,  2.2,  2.4,  2.8])
mag_h080_lc = np.array([  3.0,  2.8,  2.1,  2.1,  1.8,  2.2,  2.8,  2.3,  1.8,  2.0,  2.1,  2.6])
mag_h100_lc = np.array([  2.4,  1.1,  0.8,  1.2,  1.7,  1.9,  3.0,  1.8,  1.5,  1.5,  0.7,  1.1])

mag_h035_naive = np.array([  0.8,  0.9,  0.8,  0.8,  0.9,  0.9,  1.0,  0.9,  0.9,  0.9,  0.8,  0.8])
mag_h050_naive = np.array([  1.6,  0.8,  0.8,  1.0,  1.0,  1.0,  1.2,  1.0,  1.0,  1.0,  0.8,  0.8])
mag_h080_naive = np.array([  2.6,  1.0,  0.9,  0.8,  0.7,  0.8,  1.1,  0.8,  0.7,  0.5,  0.5,  1.0])
mag_h100_naive = np.array([  0.1,  0.2,  0.3,  0.3,  0.4,  0.7,  1.2,  0.7,  0.4,  0.3,  0.3,  0.2])



fig, ax = polar_chart('', 0, phase_deg, [mag_h100_lc, mag_h080_lc, mag_h050_lc, mag_h035_lc],
                      [mag_h100_naive, mag_h080_naive, mag_h050_naive, mag_h035_naive], legend = ["1.00", "0.80", "0.50", "0.35"])

plt.savefig('fig.png')
plt.savefig('pp6.pdf', backend='pgf')
#plt.savefig('fig.svg')
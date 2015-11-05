import numpy as np
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt

# Control parameters: P =0.2, I = 2.0, D = 0.

time_samples    = np.linspace(0.,0.9,10)
samples1        = 2.*np.array([0., 0.358322, 0.442637, 0.475927, 0.489364, 0.495652, 0.497595, 0.499125, 0.499599, 0.49962])
samples2        = 2.*np.array([0., 0.174935, 0.451387, 0.479997, 0.492314, 0.496089, 0.498163, 0.498879, 0.499329, 0.500115])
samples3        = 2.*np.array([0., 0.0335096, 0.356196, 0.439941, 0.473523, 0.490641, 0.49689, 0.499259, 0.498285, 0.497864])
time            = np.linspace(0.,0.9,100)
tau1            = 0.095
tau2            = 0.095
tau3            = 0.095
step_response1  = 1.-np.exp(-time/tau1)
step_response2  = 1.-np.exp(-time/tau2)
step_response3  = 1.-np.exp(-time/tau3)

plt.figure()
plt.subplot(3,1,1)
plt.plot(time_samples, samples1, 'rx', time, step_response1 )
plt.subplot(3,1,2)
plt.plot(time_samples, samples2, 'rx', time, step_response2)
plt.subplot(3,1,3)
plt.plot(time_samples, samples3, 'rx', time, step_response3)
plt.show()

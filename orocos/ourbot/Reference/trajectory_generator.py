from omgtools import *
import matplotlib.pyplot as plt
import csv
import numpy as np

# set up vehicle
vehicle = Holonomic(shapes=Circle(0.1), options={'syslimit': 'norm_2'}, bounds={'vmin': -0.5, 'vmax': 0.5})
vehicle.set_initial_conditions([0., 0.])
vehicle.set_terminal_conditions([1., 1.])

# set up environment
environment = Environment(room={'shape': Rectangle(5., 5.), 'position': [0, 0]})

# set up problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'horizon_time': 6})
problem.init()

# simulate
vehicle.plot('input')
simulator = Simulator(problem)
trajectories1 = simulator.run_once()


pos_traj1 = trajectories1['state'][:, :]
vel_traj1 = trajectories1['input'][:, :]
theta_traj1 = np.linspace(0, 0, pos_traj1.shape[1])


vehicle.set_terminal_conditions([0.5, 0.5])
problem.reinitialize()

trajectories2 = simulator.run_once()
pos_traj2 = trajectories2['state'][:, :]
vel_traj2 = trajectories2['input'][:, :]
theta_traj2 = np.linspace(0, 0, pos_traj2.shape[1])


pos_traj = np.c_[pos_traj1, pos_traj2]
vel_traj = np.c_[vel_traj1, vel_traj2]
theta_traj = np.r_[theta_traj1, theta_traj2]


pos_traj = np.c_[np.c_[pos_traj[:, 0]]*np.ones(500), pos_traj]
vel_traj = np.c_[np.c_[vel_traj[:, 0]]*np.ones(500), vel_traj]
theta_traj = np.r_[theta_traj[0]*np.ones(500), theta_traj]

n = pos_traj.shape[1]

# save trajectory in csv-format
data = np.c_[pos_traj.T, theta_traj.T, vel_traj.T, np.zeros(n)]

np.savetxt('example_trajectory.csv', data, delimiter=',')

plt.figure()
plt.hold(True)
plt.plot(data[:, 0])
plt.plot(data[:, 1])
plt.plot(data[:, 2])

plt.figure()
plt.hold(True)
plt.plot(data[:, 3])
plt.plot(data[:, 4])
plt.plot(data[:, 5])

plt.show(block=True)

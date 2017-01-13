
import sys, os
sys.path.insert(0, '/home/ruben/Documents/Work/Repositories/casadi_binary_old/')
sys.path.insert(0, '/home/ruben/Documents/Work/Programs/motionplanningtoolbox/')
from omgtools import *

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create vehicle
options = {'safety_distance': 0.05}
N = 3
rect = Rectangle(0.55, 0.4)
rect.radius = 0.02
vehicles = [Holonomic(shapes=rect, options=options, bounds={'vmin': -0.5, 'vmax': 0.5, 'amin': -0.5, 'amax': 0.5}) for _ in range(N)]
fleet = Fleet(vehicles)
configuration = np.array([[-0.2, -0.2], [-0.2, 0.2], [0.4, 0]])
# configuration = np.array([[-0.2, -0.2], [-0.2, 0.2]])
init_positions = [0.8, 0.8] + configuration
terminal_positions = [3.2, 1.5] + configuration

# init_positions = [0.8, 1.125] + configuration
# terminal_positions = [3.5, 1.125] + configuration


fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())


# create environment
# width = 4.0
# height = 2.25

width = 4.61
height = 2.59

environment = Environment(room={'shape': Rectangle(width, height), 'position': [0.5*width, 0.5*height]})
rectangle = Rectangle(width=0.3, height=0.8)

environment.add_obstacle(Obstacle({'position': [0.5*width, 0.5*height]}, shape=rectangle))
# environment.add_obstacle(Obstacle({'position': [2.5, 2]}, shape=rectangle))

# create a point-to-point problem
problem = FormationPoint2point(fleet, environment)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'horizon_time': 10.})
problem.set_options({'rho': 0.2, 'init_iter': 5})
problem.set_options({'hard_term_con': True})
problem.init()

options = {}
casadi_path = os.path.join('/home/ruben/Documents/Work/Repositories/casadi_binary/')
options['directory'] = os.path.join(os.getcwd(), 'Toolbox/')
# path to object files of your exported optimization problem
options['casadiobj'] = os.path.join(options['directory'], 'bin/')
# your casadi include path
options['casadiinc'] = os.path.join(casadi_path, 'include/')
# your casadi library path
options['casadilib'] = os.path.join(casadi_path, 'casadi/')
options['namespace'] = 'omgf'

# export the problem
problem.export(options)
fleet.plot('input')
problem.plot('scene')
simulator = Simulator(problem, sample_time=0.01, update_time=0.5)
trajectories, signals = simulator.run()
problem.plot_movie('scene', repeat=True, number_of_frames=100)


# note: you need to implement your vehicle type in c++. Take a look at
# Holonomic.cpp and Holonomic.hpp which are also exported as an example.

import matplotlib.pyplot as plt
plt.show(block=True)

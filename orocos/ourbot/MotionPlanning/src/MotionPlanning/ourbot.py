import sys, os
sys.path.insert(0, '/home/ruben/Documents/Work/Programs/motionplanningtoolbox/')
from omgtools import *
import os

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create vehicle
# options = {'safety_distance': 0.05, 'safety_weight': 5}
options = {}
rect = Rectangle(0.55, 0.4)
rect.radius = 0.02
vehicle = Holonomic(shapes=rect, options=options, bounds={'vmin': -0.8, 'vmax': 0.8, 'amin': -0.5, 'amax': 0.5})
# vehicle = Holonomic(shapes=Circle(0.5), options=options)
# vehicle = Holonomic(shapes=Circle(0.3))
# vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})
# vehicle.set_options({'input_disturbance': {'fc': 0.01, 'stdev': 0.05*np.ones(2)}})
# vehicle.set_options({'stop_tol': 1.e-2})
# vehicle.set_options({'syslimit': 'norm_inf'})
vehicle.set_options({'safety_distance': 0.05})

vehicle.set_initial_conditions([0.3, 0.3])
vehicle.set_terminal_conditions([3., 1.5])

# create environment
width = 4.0
height = 2.25
environment = Environment(room={'shape': Rectangle(width, height), 'position': [0.5*width, 0.5*height]})
rectangle = Rectangle(width=0.25, height=0.5)

environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [2.5, 1.5]}, shape=rectangle))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'horizon_time': 10.})
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

# export the problem
problem.export(options)
# vehicle.plot('input')
# problem.plot('scene')
# simulator = Simulator(problem, sample_time=0.01, update_time=0.5)
# trajectories, signals = simulator.run()
# problem.plot_movie('scene', number_of_frames=100)

# note: you need to implement your vehicle type in c++. Take a look at
# Holonomic.cpp and Holonomic.hpp which are also exported as an example.

import matplotlib.pyplot as plt
plt.show(block=True)

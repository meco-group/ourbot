import sys, os
from omgtools import *

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create vehicle
options = {}
rect = Rectangle(0.55, 0.4)
rect.radius = 0.02
vehicle = Holonomic(shapes=rect, options=options, bounds={'vmin': -0.5, 'vmax': 0.5, 'amin': -0.3, 'amax': 0.3})
vehicle.set_options({'safety_distance': 0.05, 'room_constraints': False})

vehicle.set_initial_conditions([0.3, 0.3])
vehicle.set_terminal_conditions([3., 1.5])

# create environment
width = 4.61
height = 2.59
environment = Environment(room={'shape': Rectangle(width, height), 'position': [0.5*width, 0.5*height]})
rectangle = Rectangle(width=0.25, height=0.5)

# environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=rectangle))
# environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=rectangle))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'horizon_time': 10.})
problem.set_options({'hard_term_con': False})
problem.init()

options = {}
options['directory'] = os.getenv('ROS_WORKSPACE') + '/ourbot/MotionPlanning/src/MotionPlanning/Toolbox/'
options['casadiobj'] = '$(ROS_WORKSPACE)/ourbot/MotionPlanning/src/MotionPlanning/Toolbox/bin/'
options['casadiinc'] = '$(CASADI_INC)'
options['casadilib'] = '$(CASADI_LIB)'
options['namespace'] = 'p2p'

# export the problem
problem.export(options)
vehicle.plot('input')
problem.plot('scene')
simulator = Simulator(problem, sample_time=0.01, update_time=0.5)
trajectories, signals = simulator.run()
# problem.plot_movie('scene', number_of_frames=100)

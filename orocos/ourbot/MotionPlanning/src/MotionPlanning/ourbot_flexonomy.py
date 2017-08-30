import sys, os
# sys.path.insert(0, os.getenv('HOME') + '/Downloads/omg-tools/')
from omgtools import *

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create vehicle
options = {}
#rect = Rectangle(0.55, 0.4)
# rect = Rectangle(0.85, 0.4)
# rect = Rectangle(0.66, 0.4)
# rect.radius = 0.02
shape = Circle(0.34)
vehicle = Holonomic(shapes=shape, options=options, bounds={'vmin': -0.3, 'vmax': 0.3, 'amin': -0.5, 'amax': 0.5})
vehicle.set_options({'safety_distance': 0.05})

vehicle.set_initial_conditions([0.5, 0.5])
# vehicle.set_terminal_conditions([2.7, 2.2])
vehicle.set_terminal_conditions([3.8, 1.1])
# vehicle.set_terminal_conditions([1.5, 0.5])
# vehicle.set_terminal_conditions([1.0, 2.0])



# extract spline parameters
coeffs = np.zeros([len(vehicle.basis.knots)-vehicle.basis.degree-1, 2])
spline_params = {'knots': vehicle.basis.knots, 'degree': vehicle.basis.degree, 'coeffs': coeffs}

# create environment
width = 4.61
height = 2.59

environment = Environment(room={'shape': Rectangle(width, height), 'position': [0.5*width, 0.5*height]})
# robot = Rectangle(width=0.25, height=0.5)
robot = Rectangle(width=2., height=2.)
environment.add_obstacle(Obstacle({'position': [width, height]}, shape=robot))
obstacle = Obstacle({'position': [2.5, 0.5]}, shape=Circle(0.34))
environment.add_obstacle(obstacle)
obstacle.set_options({'spline_traj': True})
obstacle.set_options({'spline_params': spline_params})

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
options['namespace'] = 'omg'

# export the problem
problem.export(options)
vehicle.plot('input')
problem.plot('scene')
simulator = Simulator(problem, sample_time=0.01, update_time=0.5)
trajectories, signals = simulator.run()
# problem.plot_movie('scene', number_of_frames=100)

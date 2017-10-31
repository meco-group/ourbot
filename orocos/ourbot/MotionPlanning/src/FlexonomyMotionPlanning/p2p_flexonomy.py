import os
from omgtools import *

# create vehicle
options = {}
shape = Circle(0.35)
vehicle = Holonomic(shapes=shape, options=options, bounds={'vmin': -0.3, 'vmax': 0.3, 'amin': -0.5, 'amax': 0.5})
vehicle.set_options({'safety_distance': 0.1, 'room_constraints': False, 'stop_tol': 1e-2})

A = [ 3.2, 2.]
B = [4.2, 1.2]
C = [3.2, 0.5]
D = [1., 1.2]

vehicle.set_initial_conditions(A)
vehicle.set_terminal_conditions(C)



# extract spline parameters
coeffs = np.zeros([len(vehicle.basis.knots)-vehicle.basis.degree-1, 2])
spline_params = {'knots': vehicle.basis.knots, 'degree': vehicle.basis.degree, 'coeffs': coeffs}

# create environment
width = 4.61
height = 2.59

environment = Environment(room={'shape': Rectangle(width, height), 'position': [0.5*width, 0.5*height]})
robot_arm = Rectangle(width=2., height=2.)
environment.add_obstacle(Obstacle({'position': [width, height]}, shape=robot_arm))
neighbor = Obstacle({'position': C}, shape=Circle(0.34))
neighbor.set_options({'spline_traj': True})
neighbor.set_options({'spline_params': spline_params})
environment.add_obstacle(neighbor)

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'horizon_time': 10.})
problem.set_options({'hard_term_con': False})
problem.init()

options = {}
options['directory'] = os.getenv('ROS_WORKSPACE') + '/ourbot/MotionPlanning/src/FlexonomyMotionPlanning/Toolbox/'
options['casadiobj'] = '$(ROS_WORKSPACE)/ourbot/MotionPlanning/src/FlexonomyMotionPlanning/Toolbox/bin/'
options['casadiinc'] = '$(CASADI_INC)'
options['casadilib'] = '$(CASADI_LIB)'
options['namespace'] = 'p2pflex'

# export the problem
problem.export(options)
vehicle.plot('input')
problem.plot('scene')
simulator = Simulator(problem, sample_time=0.01, update_time=0.5)
trajectories, signals = simulator.run()
# problem.plot_movie('scene', number_of_frames=100)

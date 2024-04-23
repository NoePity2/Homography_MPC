import casadi as opt
import numpy as np


class Pose(object):
	"""docstring for QPOrientation"""
	def __init__(self,params_position,params_orientation,N=15,T=0.1):
		super(Pose, self).__init__()
		self.params_position    = params_position
		self.params_orientation = params_orientation
		self.N      = N
		self.T      = T
	
	def solve(self,Tethas_ref,theta_k,state_vector,X_ref,Y_ref,is_obstacle=False,center_obstacle=None,obstacle_radius=None):

		params_orientation = self.params_orientation #gains, constraints values
		params_position    = self.params_position   
		N      = self.N
		T      = self.T

		# Objective function
		J = 0

		# Variables
		U = []

		# Variable bounds
		lbU = []
		ubU = []

		# Constraints
		g = []

		# Constraint bounds
		lbg = []
		ubg = []

		# Obstacle´s Constraint
		gl = []

		sum_x = 0
		sum_y = 0
		sum_Omega = 0
		# Loop over all control horizon
		for i in range(0, N):
			# Create variables for the angular velocities
			omega_i = opt.SX.sym('omega_' + str(i))

			# Create variables for the linear velocities
			v_i = opt.SX.sym('v_' + str(i))

			# Add to the list of variables
			U += [v_i, omega_i]

			lbU += [params_position['lbv'], params_orientation['lbO']]
			ubU += [params_position['ubv'], params_orientation['ubO']]

			# cost function
			sum_Omega += omega_i
			theta_kn  = theta_k + T*sum_Omega   

			J += ( params_orientation['alpha']*omega_i**2 + params_orientation['beta']*(theta_kn - Tethas_ref[i])**2 )
							
			g.append( theta_kn  )
			lbg.append( params_orientation['delta_min'] )
			ubg.append( params_orientation['delta_max'] )

			# cost function
			sum_x += np.cos(theta_kn) * v_i
			x_kn  = state_vector[0] + T*sum_x
			sum_y += np.sin(theta_kn) * v_i
			y_kn  = state_vector[1] + T*sum_y   

			J += ( params_position['alpha1']*v_i**2 + params_position['alpha2']*(x_kn - X_ref[i])**2 + params_position['alpha3']*(y_kn - Y_ref[i])**2 )
			
			# Obstacle´s constraints
			if(is_obstacle):
				d = np.sqrt( (x_kn - center_obstacle[0])**2 + (y_kn - center_obstacle[1])**2 )
				J += 1/( 1 + np.exp(2.25*(d - 0.05)) )
				#g.append(  (x_kn - center_obstacle[0])**2 + (y_kn - center_obstacle[1])**2 )
				#lbg.append( (obstacle_radius)**2 )
				#ubg.append( opt.inf )

			 
		# Formulate QP
		sqp = {'x':opt.vertcat(*U), 'f':J, 'g':opt.vertcat(*g)}

		# Solve with qpoases
		#solver   = opt.qpsol('solver', 'qpoases', sqp)
		solver   = opt.nlpsol('S', 'ipopt', sqp)
		solution = solver(lbx=lbU, ubx=ubU, lbg=lbg, ubg=ubg)

		self.V_opt = solution['x'][0::2]
		self.Omega_opt = solution['x'][1::2]
		self.min_function_value = solution['f']

	def get_Vopt_solution(self):
		return self.V_opt 

	def get_Omega_opt_solution(self):
		return self.Omega_opt

	def get_min_fuction_value(self):
		return self.min_function_value





import casadi as opt
import numpy as np


class Orientation(object):
	"""docstring for QPOrientation"""
	def __init__(self,params,N=15,T=0.1):
		super(Orientation, self).__init__()
		self.params = params
		self.N      = N
		self.T      = T
	
	def solve(self,Tethas_ref,theta_k):

		params = self.params #gains, constraints values
		N      = self.N
		T      = self.T

		# Objective function
		J = 0

		# Variables
		Omega = []

		# Variable bounds
		lbO = []
		ubO = []

		# Constraints
		g = []

		# Constraint bounds
		lbg = []
		ubg = []

		sum_Omega = 0
		# Loop over all control horizon
		for i in range(0, N):
		  # Create variables for the velocities
		  omega_i = opt.SX.sym('omega_' + str(i))
		  
		  # Add to the list of variables
		  Omega += [omega_i]

		  lbO += [params['lbO']]
		  ubO += [params['ubO']]

		  # cost function
		  sum_Omega += omega_i
		  theta_kn  = theta_k + T*sum_Omega   
		  
		  J += ( params['alpha']*omega_i**2 + params['beta']*(theta_kn - Tethas_ref[i])**2 )

		  # Omnidirectional constraints
		  g.append( theta_kn )
		  lbg.append( params['delta_min'] )
		  ubg.append( params['delta_max'] )

		# Formulate QP
		orientation_qp = {'x':opt.vertcat(*Omega), 'f':J, 'g':opt.vertcat(*g)}

		# Solve with qpoases
		solver   = opt.nlpsol('S', 'ipopt', orientation_qp)
		#solver   = opt.qpsol('solver', 'qpoases', orientation_qp)
		solution = solver(lbx=lbO, ubx=ubO, lbg=lbg, ubg=ubg)
		self.Omega_opt = solution['x']
		self.min_function_value = solution['f']

	def get_solution(self):
		return self.Omega_opt 

	def get_min_fuction_value(self):
		return self.min_function_value

class Position(object):
	"""docstring for Position"""
	def __init__(self,params,N=15,T=0.1):
		super(Position, self).__init__()
		self.params = params
		self.N      = N
		self.T      = T

	def solve(self,state_vector,Theta,X_ref,Y_ref,is_obstacle=False,center_obstacle=None,obstacle_radius=None):

		params = self.params #gains, constraints values
		N      = self.N
		T      = self.T		

		# Objective function
		J = 0

		# Variables
		V = []

		# Variable bounds
		lbv = []
		ubv = []

		# Constraints
		g = []

		# Constraint bounds
		lbg = []
		ubg = []


		sum_x = 0
		sum_y = 0
		# Loop over all control horizon
		for i in range(0, N):
			# Create variables for the velocities
			v_i = opt.SX.sym('v_' + str(i))

			# Add to the list of variables
			V += [v_i]

			lbv += [params['lbv']]#[-0.26]
			ubv += [params['ubv']]

			# cost function
			sum_x += np.cos(Theta[i]) * v_i
			x_kn  = state_vector[0] + T*sum_x
			sum_y += np.sin(Theta[i]) * v_i
			y_kn  = state_vector[1] + T*sum_y   

			J += ( params['alpha1']*v_i**2 + params['alpha2']*(x_kn - X_ref[i])**2 + params['alpha3']*(y_kn - Y_ref[i])**2 )
			
			# Obstacle´s constraints
			if(is_obstacle):
				g.append( (x_kn - center_obstacle[0])**2 + (y_kn - center_obstacle[1])**2  )
				lbg.append( (obstacle_radius)**2 )
				ubg.append( opt.inf )		 


		# Formulate QP
		qp = {'x':opt.vertcat(*V), 'f':J, 'g':opt.vertcat(*g)}

		# Solve with IPOPT
		solver   = opt.nlpsol('S', 'ipopt', qp)
		#solver = opt.qpsol('solver', 'qpoases', qp)

		# Get the optimal solution
		solution = solver(lbx=lbv, ubx=ubv,lbg=lbg, ubg=ubg)
		self.V_opt = solution['x']
		self.min_function_value = solution['f']

	def get_solution(self):
		return self.V_opt

	def get_min_fuction_value(self):
		return self.min_function_value



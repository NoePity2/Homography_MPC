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

class HomographyMPC:
	# def __init__(self):

	def set_values(self,d=1,n=[0.0,0.0,0.0],ck_t_ct=[0.0,0.0,0.0],rk_x_rl=None,rk_y_rl=None,c_x_r=0.0,theta=0.0,phi=0.0):
		self.d  = d
		self.nx = n[0] 
		self.ny = n[1]
		self.nz = n[2]
		self.ck_x_ct = ck_t_ct[0]
		self.ck_z_ct = ck_t_ct[2]
		self.rk_x_rl = rk_x_rl #This is a casadi variable
		self.rk_y_rl = rk_y_rl # This is a casadi variable
		self.c_x_r   = c_x_r
		self.theta   = theta
		self.phi     = phi
		self.nx_d    = self.nx/self.d
		self.ny_d    = self.ny/self.d
		self.nz_d    = self.nz/self.d
		self.a_row_1 = np.sin(2*self.theta)*np.sin(self.phi) - np.cos(2*self.theta)*np.cos(self.phi)
		self.b_row_1 = np.sin(2*self.theta)*np.cos(self.phi) + np.cos(2*self.theta)*np.sin(self.phi)
		self.c_row_1 = self.c_x_r*( np.sin(self.phi + self.theta) - np.sin(self.phi) )
		self.d_row_1 = self.d*(self.ck_x_ct*np.sin(self.phi) + self.ck_z_ct*np.cos(self.phi))
		self.a_row_3 = -np.sin(2*self.theta)*np.cos(self.phi) - np.cos(2*self.theta)*np.sin(self.phi)
		self.b_row_3 = np.sin(2*self.theta)*np.sin(self.phi) - np.cos(2*self.theta)*np.cos(self.phi)
		self.c_row_3 = self.c_x_r*( np.cos(self.phi + self.theta) - np.cos(self.phi) )
		self.d_row_3 = self.d*(-self.ck_x_ct*np.cos(self.phi) + self.ck_z_ct*np.sin(self.phi))

	def get_predictive_state_values(self,v,omega,state_vector,T):
		x_k     = state_vector[0]
		y_k     = state_vector[1]
		theta_k = state_vector[0]

		x_kn      = x_k + T*np.cos(theta_k) * v #Get the next x
		y_kn      = y_k + T*np.sin(theta_k) * v #Get the next y
		theta_kn  = theta_k + T*omega   #Get the next theta

		return x_kn, y_kn, theta_kn
	
	
class Homography11(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 1.0

	def get_h11(self):
		return self.nx_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1 + self.c_x_r*self.c_row_1 + self.d_row_1) + np.cos(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)
		
		val1 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val2 = np.sin(2*theta_kn)*np.cos(self.phi) + np.cos(2*theta_kn)*np.sin(self.phi)
		val3 = self.c_x_r*( np.sin(self.phi + theta_kn) - np.sin(self.phi) )
		return self.nx_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_1 ) + np.cos(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value

class Homography12(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h12(self):
		return self.ny_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1 + self.c_x_r*self.c_row_1 + self.d_row_1)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		val1 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val2 = np.sin(2*theta_kn)*np.cos(self.phi) + np.cos(2*theta_kn)*np.sin(self.phi)
		val3 = self.c_x_r*( np.sin(self.phi + theta_kn) - np.sin(self.phi) )
		return self.ny_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_1)
	
	def get_expected_value(self):
		return self.expected_value

class Homography13(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h13(self):
		return self.nz_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1 + self.c_x_r*self.c_row_1 + self.d_row_1) + np.sin(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		val1 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val2 = np.sin(2*theta_kn)*np.cos(self.phi) + np.cos(2*theta_kn)*np.sin(self.phi)
		val3 = self.c_x_r*( np.sin(self.phi + theta_kn) - np.sin(self.phi) )
		return self.nz_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_1) + np.sin(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value

class Homography31(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h31(self):
		return self.nx_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3 + self.c_x_r*self.c_row_3 + self.d_row_3) - np.sin(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		val1 = -np.sin(2*theta_kn)*np.cos(self.phi) - np.cos(2*theta_kn)*np.sin(self.phi)
		val2 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val3 = self.c_x_r*( np.cos(self.phi + theta_kn) - np.cos(self.phi) )
		return self.nx_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_3) - np.sin(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value

class Homography32(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h32(self):
		return self.ny_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3 + self.c_x_r*self.c_row_3 + self.d_row_3)

	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		val1 = -np.sin(2*theta_kn)*np.cos(self.phi) - np.cos(2*theta_kn)*np.sin(self.phi)
		val2 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val3 = self.c_x_r*( np.cos(self.phi + theta_kn) - np.cos(self.phi) )
		return 	self.ny_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_3)
	
	def get_expected_value(self):
		return self.expected_value
	
class Homography33(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 1.0

	def get_h33(self):
		return self.nz_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3 + self.c_x_r*self.c_row_3 + self.d_row_3) + np.cos(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		val1 = -np.sin(2*theta_kn)*np.cos(self.phi) - np.cos(2*theta_kn)*np.sin(self.phi)
		val2 = np.sin(2*theta_kn)*np.sin(self.phi) - np.cos(2*theta_kn)*np.cos(self.phi)
		val3 = self.c_x_r*( np.cos(self.phi + theta_kn) - np.cos(self.phi) )
		return self.nz_d*( rk_x_rl*val1 + rk_y_rl*val2 + self.c_x_r*val3 + self.d_row_3) + np.cos(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value 

	
class OptimizationHMPC(object):
	"""docstring for QPOrientation"""
	def __init__(self,params_position,params_orientation,vision_parameters,N=15,T=0.1):
		super(OptimizationHMPC, self).__init__()
		self.params_position    = params_position
		self.params_orientation = params_orientation
		self.vision_parameters  = vision_parameters
		self.N    = N
		self.T    = T		
		self.Homography11 = Homography11()
		self.Homography12 = Homography12()
		self.Homography13 = Homography13()
		self.Homography31 = Homography31()
		self.Homography32 = Homography32()
		self.Homography33 = Homography33()
		self.h11_desired_value = [1.0]
		self.h12_desired_value = [0.0]
		self.h13_desired_value = [0.0]
		self.h31_desired_value = [0.0]
		self.h32_desired_value = [0.0]
		self.h33_desired_value = [1.0]
		self.h11_predicted = 1.0
		self.h12_predicted = 0.0
		self.h13_predicted = 0.0
		self.h31_predicted = 0.0
		self.h32_predicted = 0.0
		self.h33_predicted =1.0
		self.V_opt = 0.0
		self.Omega_opt = 0.0

	def update_predictive_values(self,v,omega,state_vector,T):
		#Find predicted values to introduce them into Objective Function \bar{H}- \Gamma
		self.h11_predicted = self.Homography11.predicted_value(v,omega,state_vector,T)
		self.h12_predicted = self.Homography12.predicted_value(v,omega,state_vector,T)
		self.h13_predicted = self.Homography13.predicted_value(v,omega,state_vector,T)
		self.h31_predicted = self.Homography31.predicted_value(v,omega,state_vector,T)
		self.h32_predicted = self.Homography32.predicted_value(v,omega,state_vector,T)
		self.h33_predicted = self.Homography33.predicted_value(v,omega,state_vector,T)

	def set_desired_values(self,H_actual):
		self.h11_desired_value = self.Homography11.get_expected_value() - (H_actual[0] - self.h11_predicted)
		self.h12_desired_value = self.Homography12.get_expected_value() - (H_actual[1] - self.h12_predicted)
		self.h13_desired_value = self.Homography13.get_expected_value() - (H_actual[2] - self.h13_predicted)
		self.h31_desired_value = self.Homography31.get_expected_value() - (H_actual[6] - self.h31_predicted)
		self.h32_desired_value = self.Homography32.get_expected_value() - (H_actual[7] - self.h32_predicted)
		self.h33_desired_value = self.Homography33.get_expected_value() - (H_actual[8] - self.h33_predicted)

	
	def solve(self,state_vector,d=1,n=[1.0,1.0,1.0],ck_t_ct=[0.0,0.0,0.0],c_x_r=0.0,phi=0.0): #is_obstacle=False,center_obstacle=None,obstacle_radius=None):

		params_orientation = self.params_orientation #gains, constraints values
		params_position    = self.params_position  
		vision_parameters  = self.vision_parameters 
		N      = self.N
		T      = self.T
		
		#Init the state vector to zero due to the optimización is local at robot frame.
		# state_vector[0] = 0
		# state_vector[1] = 0
		# theta_k         = 0
		x_k     = state_vector[0]
		y_k     = state_vector[1]
		theta_k = state_vector[2]

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

		#Orientation references
		Theta_ref = self.get_thetas_ref(phi)

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

			#Orientation cost function
			sum_Omega += omega_i
			theta_kn  = theta_k + T*sum_Omega   

			J += params_orientation['beta']*(theta_kn - Theta_ref[i])**2 

			#Vision cost function
			sum_x += np.cos(theta_kn) * v_i
			x_kn  = x_k + T*sum_x
			sum_y += np.sin(theta_kn) * v_i
			y_kn  = y_k + T*sum_y

			if(n == [] or ck_t_ct == [] ):
				n=[1.0,1.0,1.0]
				ck_t_ct=[0.0,0.0,0.0]
				d = 1

			self.Homography11.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)
			self.Homography12.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)
			self.Homography13.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)		
			self.Homography31.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)
			self.Homography32.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)
			self.Homography33.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,theta=theta_kn,phi=phi)   

			J += vision_parameters['b11']*(self.Homography11.get_h11() - self.h11_desired_value[0] )**2  
			J += vision_parameters['b12']*(self.Homography12.get_h12() - self.h12_desired_value[0] )**2
			J += vision_parameters['b13']*(self.Homography13.get_h13() - self.h13_desired_value[0] )**2
			J += vision_parameters['b31']*(self.Homography31.get_h31() - self.h31_desired_value[0] )**2
			J += vision_parameters['b32']*(self.Homography32.get_h32() - self.h32_desired_value[0] )**2
			J += vision_parameters['b33']*(self.Homography33.get_h33() - self.h33_desired_value[0] )**2
							
			g.append( theta_kn  )
			lbg.append( params_orientation['delta_min'] )
			ubg.append( params_orientation['delta_max'] )

			J += ( params_position['alpha_v']*v_i**2 ) + ( params_orientation['alpha_w']*omega_i**2 )  #+ params_position['alpha2']*(x_kn - X_ref[i])**2 + params_position['alpha3']*(y_kn - Y_ref[i])**2 )
			
			# Obstacle´s constraints
			# if(is_obstacle):
			# 	d = np.sqrt( (x_kn - center_obstacle[0])**2 + (y_kn - center_obstacle[1])**2 )
			# 	J += 1/( 1 + np.exp(2.25*(d - 0.05)) )
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
	
	def get_predicted_values(self):
		
		return self.h11_predicted, self.h12_predicted, self.h13_predicted, self.h31_predicted, self.h32_predicted, self.h33_predicted
		
	def get_rk_theta_rk1(self,w,state_vector):
		return state_vector[2] + self.T * w

	def get_thetas_ref(self,phi):
		return np.linspace(phi/(self.N), phi, num=self.N)
		
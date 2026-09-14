import casadi as opt
import numpy as np
import time


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

		x_x = opt.SX.sym('x_x')
		x_y = opt.SX.sym('x_y')
		x_theta = opt.SX.sym('x_theta')

		x_ref = opt.SX.sym('x_ref')
		y_ref = opt.SX.sym('y_ref')
		theta_ref = opt.SX.sym('theta_ref')

		u_v = opt.SX.sym('u_v')
		u_w = opt.SX.sym('u_w')

		x   = opt.vertcat(x_x,x_y,x_theta)
		x_r = opt.vertcat(x_ref,y_ref,theta_ref)
		u   = opt.vertcat(u_v, u_w)

		# DDR's kinematic model
		xdot = opt.vertcat( u_v*opt.cos(x_theta), u_v*opt.sin(x_theta), u_w )

		# Objective term
		L = params_orientation['alpha_w']*u_w**2 + params_orientation['beta']*(x_theta - theta_ref)**2 + params_position['alpha_v']*u_v**2 + params_position['alpha2']*(x_x - x_ref)**2 + params_position['alpha3']*(x_y - y_ref)**2

		# Fixed step Euler integrator
		f = opt.Function('f', [x, u, x_r], [xdot, L])
		X0 = opt.SX.sym('X0', 3)
		XR = opt.SX.sym('XR', 3)
		U = opt.SX.sym('U',2)
		X = X0
		Q = 0
		k1, k1_q = f(X, U, XR)
		X = X + T*k1
		Q = Q + T*k1_q
	
		F = opt.Function('F', [X0, U, XR], [X, Q],['x0','u','xr'],['xf','qf'])

		# Start with an empty NLP
		w   = []
		w0  = []
		lbw = []
		ubw = []
		J   = 0
		g   = []
		lbg = []
		ubg = []

		# initial conditions
		Xk = opt.SX.sym('X0', 3)
		w += [Xk]
		lbw += [state_vector[0], state_vector[1], theta_k]
		ubw += [state_vector[0], state_vector[1], theta_k ]
		# Loop over all control horizon

		# Formulate the NLP
		for k in range(N):
		    # Create variables for the angular velocities
		    omega_i = opt.SX.sym('omega_' + str(k))

		    # Create variables for the linear velocities
		    v_i = opt.SX.sym('v_' + str(k))

		    # Add to the list of variables
		    Uk = opt.vertcat(v_i, omega_i)
		    w   += [Uk]
		    lbw += [params_position['lbv'], params_orientation['lbO']]
		    ubw += [params_position['ubv'], params_orientation['ubO']]

		    # Integrate till the end of the interval
		    Fk = F(x0=Xk, u=Uk, xr=[X_ref[k], Y_ref[k], Tethas_ref[k]])
		    Xk_end = Fk['xf']
		    J=J+Fk['qf']

		    # New NLP variable for state at end of interval
		    Xk = opt.SX.sym('X_' + str(k+1), 3)
		    w   += [Xk]
		    lbw += [-opt.inf, -opt.inf, -opt.inf]
		    ubw += [ opt.inf,  opt.inf,  opt.inf]

		    # if(k==(N-1)):
		    # 	J = J + 10000*( (Xk[0] - X_ref[k])**2 + (Xk[1] - Y_ref[k])**2 + (Xk[2] - Tethas_ref[k])**2 )

		    # Add equality constraint
		    g   += [Xk_end-Xk]
		    lbg += [0, 0, 0]
		    ubg += [0, 0, 0]

		    if(is_obstacle):
		    	distance = np.sqrt( (Xk[0] - center_obstacle[0][0])**2 + (Xk[1] - center_obstacle[0][1])**2 )
		    	F_max   = 20
		    	a       = 100
		    	b       = 0.5
		    	c       = 1
		    	exp_arg = a*( c*distance - b - obstacle_radius )
		    	den = 1 + np.exp( exp_arg )
		    	J += (F_max)/(den)
		    	distance = np.sqrt( (Xk[0] - center_obstacle[1][0])**2 + (Xk[1] - center_obstacle[1][1])**2 )
		    	exp_arg = a*( c*distance - b - obstacle_radius )
		    	den = 1 + np.exp( exp_arg )
		    	J += (F_max)/(den)


			 
		# Create an NLP solver
		prob = {'f': J, 'x': opt.vertcat(*w), 'g': opt.vertcat(*g)}
		#solver = opt.qpsol('solver', 'qpoases', prob)
		solver = opt.nlpsol('solver', 'ipopt', prob)

		# Solve the NLP
		solution = solver(lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

		self.V_opt = solution['x'][3::5]
		self.Omega_opt = solution['x'][4::5]
		self.min_function_value = solution['f']

	def get_Vopt_solution(self):
		return self.V_opt 

	def get_Omega_opt_solution(self):
		return self.Omega_opt

	def get_min_fuction_value(self):
		return self.min_function_value


class HomographyMPC:
	# def __init__(self):

	def set_values(self,d=1,n=[0.0,0.0,0.0],ck_t_ct=[0.0,0.0,0.0],rk_x_rl=None,rk_y_rl=None,c_x_r=0.0,c_z_r=0.0,theta=0.0,phi=0.0):
		self.d  = d
		self.nx = n[0] 
		self.ny = n[1]
		self.nz = n[2]
		self.ck_x_ct = ck_t_ct[0]
		self.ck_z_ct = ck_t_ct[2]
		self.rk_x_rl = rk_x_rl #This is a casadi variable
		self.rk_y_rl = rk_y_rl # This is a casadi variable
		self.c_x_r   = c_x_r
		self.c_z_r   = c_z_r

		self.theta   = theta
		self.phi     = phi
		self.nx_d    = self.nx/self.d
		self.ny_d    = self.ny/self.d
		self.nz_d    = self.nz/self.d

		self.a_row_1 = -np.sin(self.phi) 
		self.b_row_1 = -np.cos(self.phi) 
		self.d_row_1 = self.d*(-self.ck_x_ct*np.cos(self.phi) + self.ck_z_ct*np.sin(self.phi)) 

		self.a_row_3 = np.cos(self.phi) 
		self.b_row_3 = -np.sin(self.phi) 
		self.d_row_3 = self.d*(-self.ck_x_ct*np.sin(self.phi) - self.ck_z_ct*np.cos(self.phi))  


	def get_predictive_state_values(self,v,omega,state_vector,T):
		x_k     = state_vector[0]
		y_k     = state_vector[1]
		theta_k = state_vector[2]

		x_kn      = x_k + T*np.cos(theta_k) * v #Get the next x
		y_kn      = y_k + T*np.sin(theta_k) * v #Get the next y
		theta_kn  = theta_k + T*omega   #Get the next theta

		return x_kn, y_kn, theta_kn
	
	
class Homography11(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 1.0

	def get_h11(self):
		return self.nx_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1 +  self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + self.theta)) + self.c_z_r*(np.sin(self.phi + self.theta) - np.sin(self.phi)) ) + np.cos(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		return self.nx_d*( -np.sin(self.phi)*rk_x_rl - np.cos(self.phi)*rk_y_rl  + self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn)) + self.c_z_r*(np.sin(self.phi + theta_kn) - np.sin(self.phi)) ) + np.cos(self.phi + theta_kn)

	def get_expected_value(self):
		return self.expected_value

class Homography12(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h12(self):
		return self.ny_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1  + self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + self.theta)) + self.c_z_r*(np.sin(self.phi + self.theta) - np.sin(self.phi)))
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		return self.ny_d*( -np.sin(self.phi)*rk_x_rl - np.cos(self.phi)*rk_y_rl  + self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn)) + self.c_z_r*(np.sin(self.phi + theta_kn) - np.sin(self.phi)) )
		
	def get_expected_value(self):
		return self.expected_value

class Homography13(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h13(self):
		return self.nz_d*( self.rk_x_rl*self.a_row_1 + self.rk_y_rl*self.b_row_1 + self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + self.theta)) + self.c_z_r*(np.sin(self.phi + self.theta) - np.sin(self.phi)) ) - np.sin(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)
		return self.nz_d*( -np.sin(self.phi)*rk_x_rl - np.cos(self.phi)*rk_y_rl  + self.d_row_1 + self.c_x_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn)) + self.c_z_r*(np.sin(self.phi + theta_kn) - np.sin(self.phi)) ) - np.sin(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value

class Homography31(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h31(self):
		return self.nx_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3 + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + self.theta)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + self.theta))) + np.sin(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		return self.nx_d*( np.cos(self.phi)*rk_x_rl - np.sin(self.phi)*rk_y_rl  + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + theta_kn)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn))) + np.sin(self.phi + theta_kn)
	
	def get_expected_value(self):
		return self.expected_value

class Homography32(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 0.0

	def get_h32(self):
		return self.ny_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3  + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + self.theta)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + self.theta)))

	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		return 	self.ny_d*( np.cos(self.phi)*rk_x_rl - np.sin(self.phi)*rk_y_rl + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + theta_kn)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn)))

	
	def get_expected_value(self):
		return self.expected_value
	
class Homography33(HomographyMPC):
	def __init__(self):
		super().__init__()
		self.expected_value = 1.0

	def get_h33(self):
		return self.nz_d*( self.rk_x_rl*self.a_row_3 + self.rk_y_rl*self.b_row_3 + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + self.theta)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + self.theta))) + np.cos(self.phi + self.theta)
	
	def predicted_value(self,v,omega,state_vector,T):

		rk_x_rl,rk_y_rl,theta_kn = self.get_predictive_state_values(v,omega,state_vector,T)

		return self.nz_d*( np.cos(self.phi)*rk_x_rl - np.sin(self.phi)*rk_y_rl + self.d_row_3 + self.c_x_r*(np.sin(self.phi) - np.sin(self.phi + theta_kn)) + self.c_z_r*(np.cos(self.phi) - np.cos(self.phi + theta_kn))) + np.cos(self.phi + theta_kn)

	
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
		self.h33_predicted = 1.0
		self.V_opt = 0.0
		self.Omega_opt = 0.0
		self.Execution_Times = []

	def update_predictive_values(self,v,omega,state_vector,T):
		#Find predicted values to introduce them into Objective Function \bar{H}- \Gamma
		self.h11_predicted = self.Homography11.predicted_value(v,omega,state_vector,T)
		self.h12_predicted = self.Homography12.predicted_value(v,omega,state_vector,T)
		self.h13_predicted = self.Homography13.predicted_value(v,omega,state_vector,T)
		self.h31_predicted = self.Homography31.predicted_value(v,omega,state_vector,T)
		self.h32_predicted = self.Homography32.predicted_value(v,omega,state_vector,T)
		self.h33_predicted = self.Homography33.predicted_value(v,omega,state_vector,T)

	def set_desired_values(self,H_actual):
		print(H_actual)
		self.h11_desired_value = self.Homography11.get_expected_value() - (H_actual[0] - self.h11_predicted)
		self.h12_desired_value = self.Homography12.get_expected_value() - (H_actual[1] - self.h12_predicted)
		self.h13_desired_value = self.Homography13.get_expected_value() - (H_actual[2] - self.h13_predicted)
		self.h31_desired_value = self.Homography31.get_expected_value() - (H_actual[6] - self.h31_predicted)
		self.h32_desired_value = self.Homography32.get_expected_value() - (H_actual[7] - self.h32_predicted)
		self.h33_desired_value = self.Homography33.get_expected_value() - (H_actual[8] - self.h33_predicted)

	
	def solve(self,state_vector,d=1,n=[1.0,1.0,1.0],ck_t_ct=[0.0,0.0,0.0],c_x_r=0.0,c_z_r=0.0,phi=0.0,is_obstacle=False,c1=0.0,c2=0.0,c3=0.0,distance=0.0):

		params_orientation = self.params_orientation #gains, constraints values
		params_position    = self.params_position  
		vision_parameters  = self.vision_parameters 
		N      = self.N
		T      = self.T
		
		# #Init the state vector to zero due to the optimization is local at robot frame.
		x_k     = float(state_vector[0])
		y_k     = float(state_vector[1])
		theta_k = float(state_vector[2])

		v_prev  = opt.SX.sym('v_prev')

		# Start with an empty NLP
		w   = []
		lbw = []
		ubw = []
		J   = 0
		g   = []
		lbg = []
		ubg = []

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
			Uk = opt.vertcat(v_i, omega_i)
			w   += [Uk]
			lbw += [params_position['lbv'], params_orientation['lbO']]
			ubw += [params_position['ubv'], params_orientation['ubO']]

			#Orientation cost function
			sum_Omega += T*omega_i
			theta_kn  = theta_k + sum_Omega   

			#J += 0.5*params_orientation['beta']*(theta_kn - Theta_ref[i])**2 

			#Vision cost function
			sum_x += T*np.cos(theta_kn) * v_i
			x_kn  = x_k + sum_x
			sum_y += T*np.sin(theta_kn) * v_i
			y_kn  = y_k + sum_y

			if(n == [] or ck_t_ct == [] ):
				n=[0.0,0.0,1.0]
				ck_t_ct=[0.0,0.0,1.0]
				d = 1

			
			self.Homography11.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)
			self.Homography12.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)
			self.Homography13.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)		
			self.Homography31.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)
			self.Homography32.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)
			self.Homography33.set_values(d=d,n=n,ck_t_ct=ck_t_ct,rk_x_rl=x_kn,rk_y_rl=y_kn,c_x_r=c_x_r,c_z_r=c_z_r,theta=theta_kn,phi=phi)   

			'''lambda_k = vision_parameters['lambda_gain']**i
			J += 0.5 * lambda_k*vision_parameters['b11'] * (self.Homography11.get_h11() - self.h11_desired_value[0] )**2 
			J += 0.5 * lambda_k*vision_parameters['b12'] * (self.Homography12.get_h12() - self.h12_desired_value[0] )**2
			J += 0.5 * lambda_k*vision_parameters['b13'] * (self.Homography13.get_h13() - self.h13_desired_value[0] )**2
			J += 0.5 * lambda_k*vision_parameters['b31'] * (self.Homography31.get_h31() - self.h31_desired_value[0] )**2
			J += 0.5 * lambda_k*vision_parameters['b32'] * (self.Homography32.get_h32() - self.h32_desired_value[0] )**2
			J += 0.5 * lambda_k*vision_parameters['b33'] * (self.Homography33.get_h33() - self.h33_desired_value[0] )**2'''
			
			J += 0.5 * vision_parameters['b11'] * (self.Homography11.get_h11() - self.h11_desired_value[0] )**2 
			J += 0.5 * vision_parameters['b12'] * (self.Homography12.get_h12() - self.h12_desired_value[0] )**2
			J += 0.5 * vision_parameters['b13'] * (self.Homography13.get_h13() - self.h13_desired_value[0] )**2
			J += 0.5 * vision_parameters['b31'] * (self.Homography31.get_h31() - self.h31_desired_value[0] )**2
			J += 0.5 * vision_parameters['b32'] * (self.Homography32.get_h32() - self.h32_desired_value[0] )**2
			J += 0.5 * vision_parameters['b33'] * (self.Homography33.get_h33() - self.h33_desired_value[0] )**2
							
			# g.append( theta_kn  )
			# lbg.append( params_orientation['delta_min'] )
			# ubg.append( params_orientation['delta_max'] )

			J += ( 0.5*params_position['alpha_v']*v_i**2 ) + ( 0.5*params_orientation['alpha_w']*omega_i**2 )  
						# Delta between subsequent time steps
			if i == 0:
				delta_v = v_i
				delta_omega = omega_i
			else:
				delta_v = v_i - v_prev
				delta_omega = omega_i - omega_prev

			v_prev = v_i
			omega_prev = omega_i
			J+= 0.0*delta_v**2 #2.5
			J+= 0.0*delta_omega**2 #3.5

			
			# Obstacle´s constraints
			if(is_obstacle):
				g   += [distance*(c1*x_kn + c2*y_kn)]
				lbg += [-opt.inf]
				ubg += [c3]
				#J += 0.5*params_orientation['beta']*(theta_kn - Theta_ref[i])**2
				J+= 7.5e6*delta_omega**2 #3.5
				
			else:
				J += 0.5*params_orientation['beta']*(theta_kn - Theta_ref[i])**2

			 
		# Formulate QP
		sqp = {'x':opt.vertcat(*w), 'f':J, 'g':opt.vertcat(*g)}

		# Solve with qpoases
		#solver   = opt.qpsol('solver', 'qpoases', sqp)
		solver   = opt.nlpsol('solver', 'ipopt', sqp)

		t0_execution = time.time()
		solution = solver(lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
		tf_execution = time.time()
		SolvingTime = tf_execution - t0_execution
		self.Execution_Times.append(SolvingTime)
		#print(solution['x'])
		
		self.V_opt     = solution['x'][0::2]
		self.Omega_opt = solution['x'][1::2]
		print(solution['x'])	
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
		
		

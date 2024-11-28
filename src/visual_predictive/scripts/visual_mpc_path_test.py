#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi
import numpy as np
import casadi as opt
from std_msgs.msg import Float64MultiArray, Float64
from hmpc_utils import sigma
from mpc_ddr.mpc.nmpc import OptimizationHMPC, Homography11,Homography12,Homography13,Homography31,Homography32, Homography33
from mpc_ddr.params.params import orientation_params, position_params, vision_parameters


class VMPC():


	def __init__(self,params_position=None,params_orientation=None,vision_parameters=None,x_init=0,y_init=0,yaw_init=0,phi_target=0.0,T=0.1,N=5,c_x_r=0.0,c_z_r=-0.27):
		rospy.init_node('nonlinear_vmpc')
		rospy.loginfo('Program has started') #Beginning terminal msg
		rospy.logwarn("To start the movement, the simulation must be running\n\n")
		rate = rospy.Rate(10) #Node frequency (Hz) 
		rospy.on_shutdown(self.shutdownhook)

		#Variables of controller
		self.N          = N
		self.delta_t    = 0.0
		self.ctrl_c     = False
		self.vel_x      = 0
		self.vel_y      = 0
		self.omega      = 0
		self.T          = T
		self.x          = x_init
		self.y          = y_init
		self.c_x_r      = c_x_r #Traslation between robot with respec to the camera x-axis
		self.c_z_r      = c_z_r
		#self.PHI_TARGET = phi_target*np.pi/180
		#self.phi_k      = self.PHI_TARGET - yaw_init

        #Velocities (Controls)
		self.angular_velocity = 0.0
		self.V_k              = 0.0

		self.params_position    = params_position
		self.params_orientation = params_orientation
		self.vision_parameters  = vision_parameters

		M = 0.26
		pico_time = 0
		self.task = 0 

		#States
		#self.yaw = yaw_init*np.pi/180
		#self.x_k = np.array( [[x_init],[y_init]] )
		self.x_k = np.array( [ [x_init],[y_init],[yaw_init*np.pi/180]] )

		# Obstacle parameters
		#self.center_obstacle = (1.25, 1.0)
		#self.obstacle_raduis = 0.8
		#self.security_margin = 0.05

		#Nodos
		self.homo_sub       = rospy.Subscriber("/homograpy_numerical", Float64MultiArray,self.homograpy_callback)
		self.ck_t_ct_sub    = rospy.Subscriber("/ck_t_ct", Float64MultiArray,self.ck_t_ct_vector_callback)
		self.n_plane__sub   = rospy.Subscriber("/n_plane_vector", Float64MultiArray,self.n_vector_callback)
		self.d_value_sub    = rospy.Subscriber("/d_value",Float64,self.d_value_callback)
		self.phi_sub        = rospy.Subscriber("/phi",Float64,self.phi_value_callback)
		self.task_sub		= rospy.Subscriber("/n_task",Float64,self.task_value_callback)

		self.homo_matrix    = Float64MultiArray()
		self.ck_t_ct_vector = Float64MultiArray()
		self.n_vector       = Float64MultiArray()
		self.d_value        = Float64()
		self.phi            = Float64()

		#Homography predicted values
		self.predicted_values = []
	
		# Solvers
		self.HMPC = OptimizationHMPC(self.params_position,self.params_orientation,self.vision_parameters,self.N,self.T)

		#optimal_controls_pub = rospy.Publisher("/optimal_controls", Twist, queue_size=10)
		#self.optimal_controls = Twist()
		self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		#self.error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10)
		#self.error_msg = Pose2D()

		self.vel_msg.linear.y = self.vel_msg.linear.z = self.vel_msg.angular.x = self.vel_msg.angular.y = 0
		self.t0 = rospy.Time.now().to_sec()
		
		rospy.on_shutdown(self.shutdownhook)

		file_obj = open("data_py.txt","w+") #If only the filename is given
		
		self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time

		while not rospy.is_shutdown():
			print("Computing optimal controls ...")
			self.vel_ops() #Compute the control signals
			print("phi = ", np.rad2deg(self.phi.data))

			if self.task < 2:
				self.V_k[0] = sat_smooth_exp(self.V_k[0], 0.23, M)
				## GIRAR A LA DERECHA
				if(abs(self.homo_matrix.data[8]) < 1 and self.homo_matrix.data[6] > 0.03 and self.homo_matrix.data[2] < -0.03):
					self.angular_velocity[0] = sat_smooth_exp(self.angular_velocity[0], 0.2, M) * -1
				
				## GIRAR A LA IZQUIERDA
				if(abs(self.homo_matrix.data[8]) < 1 and self.homo_matrix.data[6] < -0.03 and self.homo_matrix.data[2] > 0.03):
					self.angular_velocity[0] = sat_smooth_exp(self.angular_velocity[0], 0.2, M)# * -1			
			else:
				if (pico_time > 2):
					# Mezclar suavemente las funciones para evitar discontinuidad
					self.V_k[0] = sigma(self.V_k[0]*2, 0.25, M)

				else:
					self.V_k[0] = M
					pico_time = pico_time + 1

				## GIRAR A LA DERECHA
				if(abs(self.homo_matrix.data[8]) < 1 and self.homo_matrix.data[6] > 0.009 and self.homo_matrix.data[2] < -0.009):
					#self.angular_velocity[0] = self.angular_velocity[0]
					self.angular_velocity[0] = sat_smooth_exp(self.angular_velocity[0], 0.08, 0.15) * -1
				
				## GIRAR A LA IZQUIERDA
				if(abs(self.homo_matrix.data[8]) < 1 and self.homo_matrix.data[6] < -0.009 and self.homo_matrix.data[2] > 0.009):
					#self.angular_velocity[0] = self.angular_velocity[0]
					self.angular_velocity[0] = sat_smooth_exp(self.angular_velocity[0], 0.08, 0.15)
				
				if(abs(self.homo_matrix.data[0])>0.99 and abs(self.homo_matrix.data[1])<0.03 and abs(self.homo_matrix.data[2])<0.03 and abs(self.homo_matrix.data[6])<0.03 and abs(self.homo_matrix.data[7])<0.03 and abs(self.homo_matrix.data[8])>0.95):
					self.vel_msg.linear.x = 0.0; self.vel_msg.angular.z = 0.0
					self.vel_pub.publish(self.vel_msg)
					self.ctrl_c = True 

			self.vel_msg.linear.x  = self.V_k[0]
			print('self.V_k[0] = ',self.V_k[0])
			V = float(self.V_k[0])
			self.vel_msg.angular.z = self.angular_velocity[0]
			print('self.angular_velocity[0] = ', self.angular_velocity[0])
			W = float(self.angular_velocity[0])
			#==================== LogCurrentResults =========================#
			self.t2 = rospy.Time.now().to_sec()
			if(self.homo_matrix.data == [] or self.phi.data == [] or self.ck_t_ct_vector.data==[] or self.n_vector.data==[] or self.d_value.data==[]):
				continue
			
			#==================== LogCurrentResults =========================#

			#Convertion from float to string (to write in a file)
			text_t = "{:.3f}".format(self.t) 
			text_h11_p = "{:.3f}".format(float(self.predicted_values[0]))
			text_h11_c = "{:.3f}".format(float(self.homo_matrix.data[0]))

			text_h12_p = "{:.3f}".format(float(self.predicted_values[1]))
			text_h12_c = "{:.3f}".format(float(self.homo_matrix.data[1]))

			text_h13_p = "{:.3f}".format(float(self.predicted_values[2]))
			text_h13_c = "{:.3f}".format(float(self.homo_matrix.data[2]))

			text_h31_p = "{:.3f}".format(float(self.predicted_values[3]))
			text_h31_c = "{:.3f}".format(float(self.homo_matrix.data[6]))

			text_h32_p = "{:.3f}".format(float(self.predicted_values[4]))
			text_h32_c = "{:.3f}".format(float(self.homo_matrix.data[7]))

			text_h33_p = "{:.3f}".format(float(self.predicted_values[5]))
			text_h33_c = "{:.3f}".format(float(self.homo_matrix.data[8]))

			text_V = "{:.3f}".format(V)
			text_W = "{:.3f}".format(W)
			text_line = text_t+"\t"+text_h11_p+"\t"+text_h11_c+"\t"+text_h12_p+"\t"+text_h12_c+"\t"+text_h13_p+"\t"+text_h13_c+"\t"+text_h31_p+"\t"+text_h31_c+"\t"+text_h32_p+"\t"+text_h32_c+"\t"+text_h33_p+"\t"+text_h33_c+"\t"+text_V+"\t"+text_W+"\n"
			#print (text_line)

			file_obj.write(text_line)

		    #================================= Update ============================================#
	  		#%%%%%%%%%%%%%%%%%% UpdatePredictiveValues(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
			self.update_predictive_values()
			#%%%%%%%%%%%%%%%%%% UpdateStates(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
			self.update_state()
			#================================= Update ============================================#
   
			self.vel_pub.publish(self.vel_msg) #Publish the control signals
			
			#%%%%%%%%%%%%%%%%%% PrepareDataForNextOptimizacion(opti_solution) %%%%%%%%%%%%%%%%%%%%#
			self.set_desired_values()
			self.reset_state() # This is due to the optimization is done with respect to the robot's frame


			#print(f"Homo current: = {self.homo_matrix.data}")
			print("Homo predicted = ", self.predicted_values)
			#self.x_k   = np.array( [[self.x],[self.y]] )
			# self.get_tracking_errors()
			self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time 
			#self.delta_t = rospy.Time.now().to_sec() - self.t2
			#print(f"delta_t = {self.delta_t}")
			
			#
			# self.error_msg.x = self.ex 
			# self.error_msg.y = self.ey #Assign the tracking errors
			# self.error_msg.theta = self.e_yaw #

			# self.error_pub.publish(self.error_msg); #Publish the tracking errors

		file_obj.close()

		print("Robot stops, but simulation keeps running\n")

	def get_tracking_errors(self):
		self.ex    = self.x - self.X_ref[0]
		self.ey    = self.y - self.Y_ref[0]
		self.e_yaw = self.yaw - self.Yaw_ref[0]


	def vel_ops(self):
		# ============ Optimization to obtain linear velocities and angular velocities in a horizon =================
		self.HMPC.solve(self.x_k,self.d_value.data,self.n_vector.data,self.ck_t_ct_vector.data,self.c_x_r,self.c_z_r,self.phi.data)#False,self.center_obstacle,self.obstacle_raduis+self.security_margin)
		self.angular_velocity = self.HMPC.get_Omega_opt_solution()
		self.V_k = self.HMPC.get_Vopt_solution()
		self.predicted_values = self.HMPC.get_predicted_values()

	def task_value_callback(self,msg):
		self.task = msg.data
	
	def homograpy_callback(self,msg):
		self.homo_matrix = msg	

	def ck_t_ct_vector_callback(self, msg):
		self.ck_t_ct_vector = msg

	def n_vector_callback(self,msg):
		self.n_vector = msg

	def d_value_callback(self,msg):
		self.d_value = msg

	def phi_value_callback(self,msg):
		self.phi = msg

	def odometry_callback(self,msg):  #CORREGIDO 
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		quater = msg.pose.pose.orientation
		quater_list = [quater.x, quater.y, quater.z, quater.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(quater_list) #A radianes

		self.vel_x = msg.twist.twist.linear.x
		self.vel_y = msg.twist.twist.linear.y
		self.omega = msg.twist.twist.angular.z
	
	def shutdownhook(self):
		self.vel_msg.linear.x = 0.0; self.vel_msg.angular.z = 0.0
		self.vel_pub.publish(self.vel_msg)
		self.ctrl_c = True

	def reset_state(self):
		self.x_k[0] = 0.0
		self.x_k[1] = 0.0
		self.x_k[2] = 0.0

	def update_predictive_values(self):
		self.HMPC.update_predictive_values(self.V_k[0],self.angular_velocity[0],self.x_k,self.T)

	def set_desired_values(self):
		self.HMPC.set_desired_values(self.homo_matrix.data)

	def update_state(self):
		x_k     = self.x_k[0]
		y_k     = self.x_k[1]
		theta_k = self.x_k[2]
		v       = self.V_k[0]
		w       = self.angular_velocity[0]
		
		self.x_k[0] = x_k + self.T * np.cos(theta_k) * v
		self.x_k[1] = y_k + self.T * np.sin(theta_k) * v
		self.x_k[2] = theta_k + self.T * w

	def get_rk_theta_rk1(self):
		return self.HMPC.get_rk_theta_rk1(self.angular_velocity[0],self.x_k)

def sat_smooth_exp(x, L_min, M):
	return L_min + (M - L_min) * np.exp(-x)




if __name__ == '__main__':
	try:
		params_position    = position_params()
		params_orientation = orientation_params()
		vision_parameters  = vision_parameters()
		PHI_TARGET         = 0.0
		VMPC(params_position=params_position,params_orientation=params_orientation,vision_parameters=vision_parameters,phi_target=PHI_TARGET)
	except rospy.ROSInterruptException:
		pass

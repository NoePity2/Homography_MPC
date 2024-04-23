#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi
import numpy as np
import casadi as opt
from std_msgs.msg import Float64MultiArray, Float64
from mpc_ddr.mpc.nmpc import OptimizationHMPC, Homography11,Homography12,Homography13,Homography31,Homography32, Homography33
from mpc_ddr.params.params import orientation_params, position_params, vision_parameters


class VMPC():


	def __init__(self,params_position=None,params_orientation=None,vision_parameters=None,x_init=0,y_init=0,yaw_init=0,T=0.1,N=15,c_x_r=0.035):
		rospy.init_node('nonlinear_vmpc')
		rospy.loginfo('Program has started') #Beginning terminal msg
		rospy.logwarn("To start the movement, the simulation must be running\n\n")
		rate = rospy.Rate(10) #Node frequency (Hz) 
		rospy.on_shutdown(self.shutdownhook)

		#Variables of controller
		self.N       = N
		self.delta_t = 0.0
		self.ctrl_c  = False
		self.vel_x   = 0
		self.vel_y   = 0
		self.omega   = 0
		self.T       = T
		self.x       = x_init
		self.y       = y_init
		self.c_x_r   = c_x_r #Traslation between robot with respecto the camera x-axis

        #Velocity
		self.Vx = 0
		self.Vy = 0

		self.params_position    = params_position
		self.params_orientation = params_orientation
		self.vision_parameters  = vision_parameters

		#States
		self.yaw   = yaw_init*np.pi/180
		self.x_k   = np.array( [[x_init],[y_init]] )
		self.x_new = np.array( [ self.x_k[0],self.x_k[1],[self.yaw]] )

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
		self.homo_matrix    = Float64MultiArray()
		self.ck_t_ct_vector = Float64MultiArray()
		self.n_vector       = Float64MultiArray()
		self.d_value        = Float64()
		self.phi            = Float64()

		# Solvers
		self.HMPC = OptimizationHMPC(self.params_position,self.params_orientation,self.vision_parameters,self.N,self.T)

		#optimal_controls_pub = rospy.Publisher("/optimal_controls", Twist, queue_size=10)
		#self.optimal_controls = Twist()
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		#self.error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10)
		#self.error_msg = Pose2D()

		self.vel_msg.linear.y = self.vel_msg.linear.z = self.vel_msg.angular.x = self.vel_msg.angular.y = 0
		self.t0 = rospy.Time.now().to_sec()
		
		rospy.on_shutdown(self.shutdownhook)

		#file_obj = open("data_py.txt","w+") #If only the filename is given
		
		while not rospy.is_shutdown():
			self.t2 = rospy.Time.now().to_sec()
			if(self.homo_matrix.data == [] or self.phi.data == [] or self.ck_t_ct_vector.data==[] or self.n_vector.data==[] or self.d_value.data==[]):
				continue
			
			if(self.homo_matrix.data[0]>0.99 and self.homo_matrix.data[1]<0.01 and self.homo_matrix.data[2]<0.01 and self.homo_matrix.data[6]<0.01 and self.homo_matrix.data[7]<0.01 and self.homo_matrix.data[8]>0.99):
				self.vel_msg.linear.x = 0.0; self.vel_msg.angular.z = 0.0
				self.vel_pub.publish(self.vel_msg)
				self.ctrl_c = True

			self.vel_ops() #Compute the control signals
			print("Computing optimal controls ...")
			print("phi = ", np.rad2deg(self.phi.data))
			self.vel_msg.linear.x  = self.V_k[0]
			print('self.V_k[0] = ',self.V_k[0])
			#V = float(self.V_k[0])
			self.vel_msg.angular.z = self.angular_velocity[0]
			print('self.angular_velocity[0] = ', self.angular_velocity[0])
			#W = float(self.angular_velocity[0])

			self.vel_pub.publish(self.vel_msg) #Publish the control signals
			#self.x_k   = np.array( [[self.x],[self.y]] )
			# self.get_tracking_errors()
			#self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time 
			#self.delta_t = rospy.Time.now().to_sec() - self.t2
			#print(f"delta_t = {self.delta_t}")
			
			#
			# self.error_msg.x = self.ex 
			# self.error_msg.y = self.ey #Assign the tracking errors
			# self.error_msg.theta = self.e_yaw #

			# self.error_pub.publish(self.error_msg); #Publish the tracking errors

			# #Convertion from float to string (to write in a file)
			# text_t = "{:.3f}".format(self.t) 
			# text_Xd = "{:.3f}".format(self.x_d)
			# text_Yd = "{:.3f}".format(self.y_d)
			# text_Thd = "{:.3f}".format(self.yaw_d*(180/np.pi))
			# text_x = "{:.3f}".format(self.x)
			# text_y = "{:.3f}".format(self.y)
			# text_theta = "{:.3f}".format(self.yaw*(180/np.pi))
			# text_V = "{:.3f}".format(V)
			# text_W = "{:.3f}".format(W)
			# text_line = text_t+"\t"+text_Xd+"\t"+text_x+"\t"+text_Yd+"\t"+text_y+"\t"+text_Thd+"\t"+text_theta+"\t"+text_V+"\t"+text_W+"\n"
			# #print (text_line)

			# file_obj.write(text_line)

		#file_obj.close()

		print("Robot stops, but simulation keeps running\n")

	def get_tracking_errors(self):
		self.ex    = self.x - self.X_ref[0]
		self.ey    = self.y - self.Y_ref[0]
		self.e_yaw = self.yaw - self.Yaw_ref[0]


	def vel_ops(self):
		# ============ Optimization to obtain linear velocities and angular velocities in a horizon =================
		self.HMPC.solve(self.yaw,self.x_k,self.homo_matrix.data,self.d_value.data,self.n_vector.data,self.ck_t_ct_vector.data,self.c_x_r,self.phi.data)#False,self.center_obstacle,self.obstacle_raduis+self.security_margin)
		self.angular_velocity = self.HMPC.get_Omega_opt_solution()
		self.V_k = self.HMPC.get_Vopt_solution()

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


if __name__ == '__main__':
	try:
		params_position    = position_params()
		params_orientation = orientation_params()
		vision_parameters  = vision_parameters()
		VMPC(params_position=params_position,params_orientation=params_orientation,vision_parameters=vision_parameters)
	except rospy.ROSInterruptException:
		pass

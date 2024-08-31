#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi
import numpy as np
import LinearFormulation as LF
from cvxopt import matrix, solvers


class Robot():


	def __init__(self,x_init=0,y_init=0,yaw_init=0,T=0.1,V=0.0,W=0.0,x_d=3.0,y_d=0.0,yaw_d=0,N=15,t=0.0,ex=0.0,ey=0.0, e_yaw=0.0):
		rospy.init_node('robot_controller_mpc')
		rospy.loginfo('Program has started') #Beginning terminal msg
		rospy.logwarn("To start the movement, the simulation must be running\n\n")
		rate = rospy.Rate(10) #Node frequency (Hz) 
		rospy.on_shutdown(self.shutdownhook)

		#Variables of controller
		self.V       = V
		self.W       = W
		self.x_d     = x_d
		self.y_d     = y_d
		self.yaw_d   = yaw_d*np.pi/180
		self.N       = N
		self.t       = t
		self.t0		 = 0.0
		self.delta_t = 0.0
		self.ctrl_c  = False
		self.vel_x   = 0
		self.vel_y   = 0
		self.omega   = 0
		self.T       = T
		self.x       = x_init
		self.y       = y_init
		self.ex      = ex
		self.ey      = ey
		self.e_yaw   = e_yaw

        #Velocity
		self.Vx = 0
		self.Vy = 0


		# Weights of the orientation problem
		self.DELTA  = 0.1 #0.1
		self.alpha  = 1e-4#1e-4
		self.beta   = 1.2e-1#1.2e-1

		# Weights of the position terms
		self.alpha1 = 1e-1 # 1e-1
		self.alpha2 = 1.2e-2 # 1.2e-1
		self.alpha3 = 1.2e-2 # 1.2e-1

		# Velocities constraints
		self.wmx    = 104.2*(np.pi/180)
		self.wmn    = -104.2*(np.pi/180)
		self.vmx    = 0.26
		self.vmn    = -0.26


		#States
		self.yaw   = yaw_init*np.pi/180
		self.x_k   = np.array( [[x_init],[y_init]] )
		self.x_new = np.array( [ self.x_k[0],self.x_k[1],[self.yaw]] )

		#References
		self.X_ref   = (x_d)*np.ones( (self.N ,1) ) # Vector
		self.Y_ref   = (y_d)*np.ones( (self.N ,1) ) # Vector
		self.Yaw_ref = (yaw_d*np.pi/180)*np.ones( (self.N,1) ) # Vector

		# State prediction
		self.Xk1  = np.zeros( (self.N,1) ) # Vector
		self.Yk1  = np.zeros( (self.N,1) ) # Vector

		self.In     = np.eye(self.N)
		self.Pput_m = LF.Pput(self.N,self.T)
		self.Ppst_m = LF.Ppst(self.N)

		self.Ppsx_m = LF.Ppsx(self.N)
		self.Ppsy_m = LF.Ppsy(self.N)


		#Nodos
		self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odometry_callback)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		self.error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10)
		self.error_msg = Pose2D()

		self.vel_msg.linear.y = self.vel_msg.linear.z = self.vel_msg.angular.x = self.vel_msg.angular.y = 0
		self.t0 = rospy.Time.now().to_sec()
		
		rospy.on_shutdown(self.shutdownhook)
		
		while not rospy.is_shutdown():
			self.t2 = rospy.Time.now().to_sec()
			print(f"time = {self.t}")
			#self.get_position_references(self.t)   
			self.vel_ops() #Compute the control signals
			self.vel_msg.linear.x  = self.V_k['x'][0]
			self.vel_msg.angular.z = self.angular_velocity['x'][0]
			self.vel_pub.publish(self.vel_msg) #Publish the control signals
			self.x_k   = np.array( [[self.x],[self.y]] )
			self.get_tracking_errors()
			self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time 
			self.delta_t = rospy.Time.now().to_sec() - self.t2
			print(f"delta_t = {self.delta_t}")
						
			#
			self.error_msg.x = self.ex 
			self.error_msg.y = self.ey #Assign the tracking errors
			self.error_msg.theta = self.e_yaw #

			self.error_pub.publish(self.error_msg); #Publish the tracking errors

	def get_tracking_errors(self):
		self.ex    = self.x - self.X_ref[0]
		self.ey    = self.y - self.Y_ref[0]
		self.e_yaw = self.yaw - self.Yaw_ref[0]


	def vel_ops(self):
		self.Qr_k    = LF.Get_Q_orientation(self.Pput_m,self.In,self.alpha,self.beta)
		self.pr_k    = LF.Get_p_orientation(self.Pput_m,self.Ppst_m,self.beta,self.yaw,self.Yaw_ref)
		self.Gr_k    = LF.Get_G_orientation(self.Pput_m,self.In)
		self.Yaw_OMN = LF.Get_Thetas_OMN(self.Xk1,self.Yk1)
		self.hr_k    = LF.Get_h_orientation(self.N,self.Ppst_m,self.yaw,self.Yaw_OMN,self.DELTA,self.wmx,self.wmn)

		#========= Solve quadratic optimization to obtain linear velocities in a horizon
		self.angular_velocity = solvers.qp(matrix(self.Qr_k), matrix(self.pr_k),G=matrix(self.Gr_k),h=matrix(self.hr_k))
		self.angles = LF.Get_angles_from_optimization(self.Ppst_m,self.yaw, self.Pput_m,self.angular_velocity['x'])
		        
		self.Ppux_m = LF.Ppux(self.N,self.T,self.angles)
		self.Ppuy_m = LF.Ppuy(self.N,self.T,self.angles)

		self.Q_k = LF.Get_Q_pose(self.Ppux_m,self.Ppuy_m,self.In,self.alpha1,self.alpha2,self.alpha3) 
		self.p_k = LF.Get_p_pose(self.Ppux_m,self.Ppuy_m,self.Ppsx_m,self.Ppsy_m,self.x_k,self.X_ref,self.Y_ref,self.alpha2,self.alpha3)
		self.G_k = LF.Get_G_pose(self.In)
		self.h_k = LF.Get_h_pose(self.N,self.vmx,self.vmn)

		self.V_k = solvers.qp(matrix(self.Q_k), matrix(self.p_k),solver='dsdp',)

		self.Xk1 = LF.Get_X_k1_vector(self.Ppsx_m,self.Ppux_m,self.x_k,self.V_k['x'])
		self.Yk1 = LF.Get_Y_k1_vector(self.Ppsy_m,self.Ppuy_m,self.x_k,self.V_k['x'])

	def get_position_references(self,t_now):
		self.X_ref   = self.x_d*np.ones( (self.N ,1) ) # Vector
		self.Y_ref   = self.y_d*np.ones( (self.N ,1) ) # Vector
		self.Yaw_ref = self.yaw_d*np.ones( (self.N ,1) ) # Vector

		t = t_now
		for i in range(self.N):
			x_d   = t/2.0
			y_d   = (t**2)/4.0
			yaw_d = np.arctan2(y_d,x_d)

			if( ( ( abs(x_d - self.x_d) < 0.1 ) and ( abs(y_d - self.y_d) < 0.1) and ( abs(yaw_d - self.yaw_d) < 0.1) )   or t_now > 4 ):
				print("Entra a la parte de referencia obtenida")
				self.X_ref[i]   = self.x_d
				self.Y_ref[i]   = self.y_d
				self.Yaw_ref[i] = self.yaw_d
			else:
				self.X_ref[i]   = x_d
				self.Y_ref[i]   = y_d
				self.Yaw_ref[i] = yaw_d

			t += self.delta_t



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
		Robot(x_d=3.0,y_d=3.0,yaw_d=45.0, yaw_init=0.0)
	except rospy.ROSInterruptException:
		pass

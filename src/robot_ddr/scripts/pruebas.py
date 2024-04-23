#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi
import do_mpc
import numpy as np
from casadi import *

class Robot():


		def __init__(self,x = 0,y=0,yaw=0,ex=0.0,ey=0.0,V=0.0,W=0.0,xd=0.0,yd=0.0,theta_d=0,t0=0.0,t=0.0):
			rospy.init_node('robot_controller')
			rospy.loginfo('Program has started') #Beginning terminal msg
			rospy.logwarn("To start the movement, the simulation must be running\n\n")
			rate = rospy.Rate(50) #Node frequency (Hz) 
			rospy.on_shutdown(self.shutdownhook)

			#Variables de instancia
			self.x = x
			self.y = y
			self.yaw = yaw
			self.ex = ex
			self.ey = ey
			self.V = V
			self.W = W
			self.xd = xd
			self.yd = yd
			self.theta_d = theta_d
			#self.t0 = t0
			self.t = t		
			self.ctrl_c = False
			self.vel_x = 0
			self.vel_y = 0
			self.omega = 0

			self.Vx = 0
			self.Vy = 0

			#Modelo continuo a discretizar con cuatro estados, dos entradas, una salida
			self.model_type = 'continuous' 
			self.model = do_mpc.model.Model(self.model_type)

			#Pesos
			self.w1  = 12.0
			self.w2  = 15.0
			self.w3  = 30.0
			self.w_v = 0.00001
			self.w_w = 0.01

			#Estados
			self.x_r = self.model.set_variable(var_type='_x', var_name='x_r', shape=(1,1))
			self.y_r = self.model.set_variable(var_type='_x', var_name='y_r', shape=(1,1))
			self.theta_r = self.model.set_variable(var_type='_x', var_name='theta_r', shape=(1,1))

			self.vecest = self.model.set_variable(var_type='_x', var_name='vecest', shape=(3,1)) #Todos los estados del robot

			#Entradas
			self.v_r = self.model.set_variable(var_type='_u', var_name='v_r',shape=(1,1))
			self.omega_r = self.model.set_variable(var_type='_u', var_name='omega_r', shape=(1,1))

			#Punto Variable en el tiempo
			self.p_d = self.model.set_variable(var_type='_tvp', var_name='p_d', shape=(3,1)) #

			#Almacenamos datos
			self.model.set_rhs('x_r', self.vecest[0])
			self.model.set_rhs('y_r', self.vecest[1])
			self.model.set_rhs('theta_r', self.vecest[2])

			#Ecuaciones del modelo
			self.vecest_sig = vertcat(     #Concatenado Vertical
				self.v_r*cos(self.theta_r),
				self.v_r*sin(self.theta_r),
				self.omega_r,
			)

			self.model.set_rhs('vecest', self.vecest_sig)

			#Configuracion del modelo
			self.model.setup()
			self.mpc = do_mpc.controller.MPC(self.model)

			setup_mpc = {
				'n_horizon': 10, #######
				't_step': 0.01, #Ajuste
				'n_robust': 0,
				'state_discretization': 'collocation',
				'cons_check_colloc_points': True,
				'nl_cons_check_colloc_points': True,
				'store_full_solution': True,
			}
			self.mpc.set_param(**setup_mpc)

			#Funcion objetivo nueva
			mterm = self.w1*0.5*(self.model.x['x_r']-self.model.tvp['p_d',0])**2 + self.w2*0.5*(self.model.x['y_r']-self.model.tvp['p_d',1])**2 + self.w3*0.5*(self.model.x['theta_r']-self.model.tvp['p_d',2])**2 
			lterm = self.w1*0.5*(self.model.x['x_r']-self.model.tvp['p_d',0])**2 + self.w2*0.5*(self.model.x['y_r']-self.model.tvp['p_d',1])**2 + self.w3*0.5*(self.model.x['theta_r']-self.model.tvp['p_d',2])**2  + self.w_v*self.v_r**2 + self.w_w*self.omega_r**2

			self.mpc.set_rterm ()
			self.mpc.set_objective(mterm=mterm, lterm=lterm)

			#Restricciones y limites
			self.mpc.bounds['lower','_u', 'v_r'] = -0.26
			self.mpc.bounds['upper','_u', 'v_r'] = 0.26
			self.mpc.bounds['upper','_u', 'omega_r'] = 1.82
			self.mpc.bounds['lower','_u', 'omega_r'] = -1.82

			#Restricción
			self.mpc.set_nl_cons('mov',-self.x_r*sin(self.theta_r) + self.y_r*cos(self.theta_r),ub = 0, soft_constraint = True)    ###Epsilon soft cons wiggle room

			self.tvp_struct_mpc = self.mpc.get_tvp_template()

			def tvp_fun_mpc(t_now):
				if (t_now < 4):
					self.tvp_struct_mpc['_tvp',:,'p_d',0] = self.xd  #t_now/2.0
					self.tvp_struct_mpc['_tvp',:,'p_d',1] = self.yd #(t_now**2)/4.0
					self.tvp_struct_mpc['_tvp',:,'p_d',2] = self.theta_d #np.arctan2((t_now**2)/4.0,t_now/2.0)
				else:
					self.tvp_struct_mpc['_tvp',:,'p_d',0] = self.xd 
					self.tvp_struct_mpc['_tvp',:,'p_d',1] = self.yd
					self.tvp_struct_mpc['_tvp',:,'p_d',2] = self.theta_d
							
				return self.tvp_struct_mpc

			self.mpc.set_tvp_fun(tvp_fun_mpc)
			self.mpc.setup()

			#Estimador - Mismo modelo
			self.estimator = do_mpc.estimator.StateFeedback(self.model)

			#Ejecución y Gráficas
			#Condiciones iniciales
			self.mpc.x0['x_r'] = x
			self.mpc.x0['y_r'] = y
			self.mpc.x0['theta_r'] = yaw

			self.x0 = self.mpc.x0

			self.estimator.x0 = self.x0
			self.mpc.set_initial_guess()


			#Nodos
			self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odometry_callback)

			self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
			self.vel_msg = Twist()
			
			self.error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10)
			self.error_msg = Pose2D()

			self.vel_msg.linear.y = self.vel_msg.linear.z = self.vel_msg.angular.x = self.vel_msg.angular.y = 0
			self.error_msg.theta = 0 #Since the orientation angle is a sub-actuated state, this field is assigned equal to zero
			self.t0 = rospy.Time.now().to_sec()

			rospy.on_shutdown(self.shutdownhook)

			while not rospy.is_shutdown():
				self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time    
				self.vel_ops() #Compute the control signals
				print("pose: ")
				print(f"x   = {self.x}")
				print(f"y   = {self.y}")
				print(f"yaw = {self.yaw}")
				self.vel_msg.linear.x  = self.V
				self.vel_msg.angular.z = self.W
				self.vel_pub.publish(self.vel_msg) #Publish the control signals
		
				self.error_msg.x = self.ex; self.error_msg.y = self.ey; #Assign the tracking errors
				self.error_pub.publish(self.error_msg); #Publish the tracking errors


		def vel_ops(self):
			x = np.array([[self.x],
				          [self.y],
				          [self.yaw],
				          [self.vel_x],
				          [self.vel_y],
				          [self.omega]
				          ])

			self.u0 = self.mpc.make_step(x)
			self.V  = self.u0[0]
			self.W  = self.u0[1]

			

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
		Robot(xd=2.0,yd=2.0,theta_d=45*(np.pi/180) )
	except rospy.ROSInterruptException:
		pass



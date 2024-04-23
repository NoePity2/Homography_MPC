#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi

class Robot():
	l = 0.1; V_max = 0.22; W_max = 2.84; T = 100; k = 0.15
	a = 1; b = 0.5; X0 = 0; Y0 = 0; w = 2*pi/T #TRAYECTORIA - LEMNISCATA

	def __init__(self,x = 0,y=0,yaw=0,ex=0.0, ey=0.0, V=0.0, W=0.0, Xd=0.0,Yd=0.0,t0=0.0,t=0.0):
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
		self.Xd = Xd
		self.Yd = Yd
		self.t0 = t0
		self.t = t		
		self.ctrl_c = False

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
			#self.t = rospy.Time.now().to_sec()-self.t0 #Compute the controller time    
			self.vel_ops() #Compute the control signals
		
			self.vel_msg.linear.x = self.V; self.vel_msg.angular.z = self.W
			self.vel_pub.publish(self.vel_msg); #Publish the control signals
	
			self.error_msg.x = self.ex; self.error_msg.y = self.ey; #Assign the tracking errors
			self.error_pub.publish(self.error_msg); #Publish the tracking errors


	def vel_ops(self):
		self.t = rospy.Time.now().to_sec()-self.t0

		self.Xd = self.X0+self.a*sin(self.w*self.t)
		self.Yd = self.Y0+self.b*sin(2*self.w*self.t)	

		self.Xdp = self.a*self.w*cos(self.w*self.t)
		self.Ydp = 2*self.b*self.w*cos(2*self.w*self.t)
		
		p_x = self.x+self.l*cos(self.yaw); 
		p_y = self.y+self.l*sin(self.yaw) #Compute the coordinates of the outside point
		self.ex = p_x-self.Xd; self.ey = p_y-self.Yd; #Compute tracking errors
	
		#Cinematic controller. Auxiliar controls, in global coordinates
		Ux = self.Xdp-self.k*self.ex; Uy = self.Ydp-self.k*self.ey
	
		#Compute the velocities according to the cinematic model for a differential type mobile robot
		self.V = Ux*cos(self.yaw)+Uy*sin(self.yaw)
		self.W = -Ux*sin(self.yaw)/self.l+Uy*cos(self.yaw)/self.l
	
		#Velocities saturation
		if (abs(self.V)>self.V_max):
			self.V = self.V_max*abs(self.V)/self.V
			print("Sat V\t") 
		if (abs(self.W)>self.W_max):
			self.W = self.W_max*abs(self.W)/self.W
			print("Sat W\t") 

		self.Xdp = self.a*self.w*cos(self.w*self.t)
		self.Ydp = 2*self.b*self.w*cos(2*self.w*self.t)
		
		p_x = self.x+self.l*cos(self.yaw); 
		p_y = self.y+self.l*sin(self.yaw) #Compute the coordinates of the outside point
		self.ex = p_x-self.Xd; self.ey = p_y-self.Yd; #Compute tracking errors
	
		#Cinematic controller. Auxiliar controls, in global coordinates
		Ux = self.Xdp-self.k*self.ex; 
		Uy = self.Ydp-self.k*self.ey

	def odometry_callback(self,msg):  #CORREGIDO 
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		quater = msg.pose.pose.orientation
		quater_list = [quater.x, quater.y, quater.z, quater.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(quater_list) #A radianes
	
	def shutdownhook(self):
		self.vel_msg.linear.x = 0.0; self.vel_msg.angular.z = 0.0
		self.vel_pub.publish(self.vel_msg)
		self.ctrl_c = True

if __name__ == '__main__':
	try:
		Robot()
	except rospy.ROSInterruptException:
		pass



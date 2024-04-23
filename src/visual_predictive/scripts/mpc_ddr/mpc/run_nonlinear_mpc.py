
import casadi as opt
import numpy as np
import matplotlib.pyplot as plt
import mpc_ddr.utils.LinearFormulation as LF
import mpc_ddr.utils.Robot as Robot
from mpc_ddr.mpc.nmpc import Pose
from mpc_ddr.params.params import orientation_params, position_params
from matplotlib.patches import Circle


T = 0.1 # Time horizon
N = 15 # number of control intervals

# =============== References values ===============

# ============== Linear parameters ==============
X_ref  = (3.0)*np.ones((N,1)) # Vector
Y_ref  = (3.0)*np.ones((N,1)) # Vector
state_vector = np.array([[0],[0]])
x  = np.zeros( (3,1) ) 


params_position = position_params()

# =============== Orientation Optimization ===============
Tethas_ref = 45*(np.pi/180)*np.ones((N,1)) # Vector
params_orientation = orientation_params()
theta_k = 0*(np.pi/180)


Pose_opt = Pose(params_position,params_orientation,N,T)


x_new = np.array( [ [0],[0],[0]] )

dt = 0.1
t0 = 0
tf = 30
times = np.arange(t0,tf,dt)
trajectory = np.zeros(( 5, len(times) )) 

center_obstacle = (2.0, 1.75)
obstacle_raduis = 0.10
security_margin = 0.05

#============= Ciclo de navegación ============
for j in range(len(times)):
   # ============ Optimization for angular velocities and linear velocities =================
   Pose_opt.solve(Tethas_ref,theta_k,state_vector,X_ref,Y_ref,True,center_obstacle,obstacle_raduis+security_margin)
   Omega_opt = Pose_opt.get_Omega_opt_solution()
   V_opt     = Pose_opt.get_Vopt_solution()

   # ========== Apply the first optimal control to a robot ======================
   u = np.array([ V_opt[0], Omega_opt[0]  ])
   x = Robot.robot_ddr(x_new,u,dt)
   print( "x = ", x )
   state_vector = np.array([  x[0] , x[1]  ]) 
   theta_k = x[2]
   x_new = x

   trajectory[0][j] = x[0]
   trajectory[1][j] = x[1]
   trajectory[2][j] = x[2]
   trajectory[3][j] = u[0]
   trajectory[4][j] = u[1]


robot = Robot.dib_robot([trajectory[0,-1],trajectory[1,-1],trajectory[2,-1]],0.05)


circle_robot    = Circle(center_obstacle, obstacle_raduis , color='r')
circle_security = Circle(center_obstacle, obstacle_raduis + security_margin, color='cyan',fill=False,linestyle='--')

fig1 = plt.figure()
ax1 = fig1.add_subplot(2,2,1)
ax1.plot(robot[:,0],robot[:,1],'r-',alpha=0.25)
ax1.plot(trajectory[0,:],trajectory[1,:])
ax1.add_patch(circle_robot)
ax1.annotate('Obstacle', xy=center_obstacle, xytext=(1.5, 0.25),
            arrowprops=dict(facecolor='black', shrink=0.05))
ax1.add_patch(circle_security)
ax1.grid()
ax1.set_xlabel("x-coordinate [m]")
ax1.set_ylabel("y-coordinate [m]")
ax1.set_aspect("equal")

ax2 = fig1.add_subplot(2,2,2)
ax2.plot(times,trajectory[0])
ax2.grid()
ax2.set_xlabel("time [s]")
ax2.set_ylabel("x-coordinate")

ax3 = fig1.add_subplot(2,2,3)
ax3.plot(times,trajectory[1])
ax3.grid()
ax3.set_xlabel("time [s]")
ax3.set_ylabel("y-coordinate")

ax4 = fig1.add_subplot(2,2,4)
ax4.plot(times,trajectory[2]*(180/np.pi))
ax4.grid()
ax4.set_xlabel("time [s]")
ax4.set_ylabel("Orientation [deg]")

fig2 = plt.figure()

ax5 = fig2.add_subplot(1,2,1)
ax5.plot(times,trajectory[3])
ax5.grid()
ax5.set_xlabel("time [s]")
ax5.set_ylabel("Linear velocity [m/s]")

ax6 = fig2.add_subplot(1,2,2)
ax6.plot(times,trajectory[4]*(180/np.pi))
ax6.grid()
ax6.set_xlabel("time [s]")
ax6.set_ylabel("Angular velocity [deg/s]")

plt.show()

print("#========== NonLinear Forumation =============")



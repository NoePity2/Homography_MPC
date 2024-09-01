import numpy as np
import matplotlib.pyplot as plt
import mpc_ddr.utils.Robot as Robot
from mpc_ddr.mpc.nmpc import Pose
from mpc_ddr.params.params import orientation_params, position_params
from matplotlib.patches import Circle

import matplotlib.patches as mpatches

from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,H_from_points,Rodrigues)

from matplotlib import rc

rc('text', usetex=True)
# plt.rcParams.update({'font.size': 12})

T = 0.1 # Time horizon
N = 15 # number of control intervals

# ============== Linear parameters ==============
X_ref  = (3)*np.ones((N,1)) # Vector
Y_ref  = (0)*np.ones((N,1)) # Vector
state_vector = np.array([[0],[0]])
x  = np.zeros( (3,1) ) 

params_position = position_params()

# =============== Orientation Optimization ===============
Tethas_ref = 0*(np.pi/180)*np.ones((N,1)) # Vector
params_orientation = orientation_params()
theta_k = 0*(np.pi/180)

Pose_opt = Pose(params_position,params_orientation,N,T)
x_new = np.array( [ [0],[0],[0]] )

dt = 0.1
t0 = 0
tf = 20
times = np.arange(t0,tf,dt)
trajectory = np.zeros(( 5, len(times) )) 
errors     = np.zeros(( 3, len(times) ))

center_obstacle_1 = (1.5, 1.4)
center_obstacle_2 = (3.0, 2.5)
center_obstacle   = [ center_obstacle_1, center_obstacle_2 ]
obstacle_raduis   = 0.15
security_margin   = 0.04

#==================== Visión terms ========================#
#Create the world points
nPoints = 4
xx       = np.array([3.5,3.5,3.5,3.5])
yy       = np.array([-0.5,0.5,0.5,-0.5])
zz       = np.array([0.5,0.5,1.5,1.5])

w_points = np.vstack([xx, yy, zz])

# Set the camera target position
angle_fixed = 90.

# Target position
target_x        = 3.0
target_y        = 0.0
target_z        = 1.0
target_roll     = np.deg2rad(0.0) # Degrees to radians 'x'
target_pitch    = np.deg2rad(angle_fixed) # Degrees to radians 'y'
target_yaw      = np.deg2rad(0.0) # Degrees to radians 'z'

camera1 = PlanarCamera() # Set the target camera
camera1.set_position(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)

p1 = camera1.projection(w_points, nPoints) # Project the points for camera 1

# Initial position
init_x_pos     = 0.0
init_y_pos     = 0.0
init_z_pos     = 1.0

init_roll    = np.deg2rad(0.0)
init_pitch   = np.deg2rad(angle_fixed)
init_yaw     = np.deg2rad(0.0)

camera2 = PlanarCamera() # Set the init camera
camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
p2 = camera2.projection(w_points, nPoints)
pixelCoordsArray  = np.zeros((2*nPoints,len(times)))   # List to save points positions on the image
averageErrorArray = np.zeros(len(times))               # Matrix to save error points positions
z_camera_Array    = np.zeros(len(times)) 

x_pos = init_x_pos
y_pos = init_y_pos
z_pos = init_z_pos

pitch = init_pitch
yaw   = init_yaw
roll  = init_roll

#============= Ciclo de navegación ============
for j in range(len(times)):
   # ============ Optimization for angular velocities and linear velocities =================
   Pose_opt.solve(Tethas_ref,theta_k,state_vector,X_ref,Y_ref,False,center_obstacle,obstacle_raduis+security_margin)
   Omega_opt = Pose_opt.get_Omega_opt_solution()
   V_opt     = Pose_opt.get_Vopt_solution()

   # ========== Apply the first optimal control to a robot ======================
   u = np.array([ V_opt[0], Omega_opt[0]  ])
   x = Robot.robot_ddr(x_new,u,dt)
   print( "x = ", x )
   state_vector = np.array([  x[0] , x[1]  ]) 
   theta_k = x[2]
   x_new = x

   x_pos = x[0, 0]
   y_pos = x[1, 0]
   yaw   = x[2, 0]
   
   camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
   p2 = camera2.projection(w_points, nPoints)

   pixelCoordsArray[:,j] = p2.reshape((2*nPoints,1), order='F')[:,0]
   pixel_error           = p2-p1
   averageErrorArray[j]  = np.mean(np.linalg.norm(pixel_error.reshape((2*nPoints,1), order='F')))
   z_camera_Array[j]     = z_pos

   trajectory[0][j] = x[0]
   trajectory[1][j] = x[1]
   trajectory[2][j] = x[2]
   trajectory[3][j] = u[0]
   trajectory[4][j] = u[1]

   errors[0][j]     = np.abs(X_ref[0] - x[0])
   errors[1][j]     = np.abs(Y_ref[0] - x[1])
   errors[2][j]     = np.abs(Tethas_ref[0] - x[2])


robot = Robot.dib_robot([trajectory[0,-1],trajectory[1,-1],trajectory[2,-1]],0.05)

# ======================================  Draw cameras ========================================

fig = plt.figure()
fig.suptitle('World setting')

ax = fig.add_subplot(1, 2, 1, projection='3d')
ax = fig.gca(projection='3d')
ax.plot(xx, yy, zz, 'o')

ax.plot(trajectory[0,:],trajectory[1,:],z_camera_Array) # Plot camera trajectory


axis_scale      = 0.5
camera_scale    = 0.09
camera1.draw_camera(ax, scale=camera_scale, color='red')
camera1.draw_frame(ax, scale=axis_scale, c='black')

camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
camera2.draw_camera(ax, scale=camera_scale, color='brown')
camera2.draw_frame(ax, scale=axis_scale, c='black')

camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
camera2.draw_camera(ax, scale=camera_scale, color='blue')
camera2.draw_frame(ax, scale=axis_scale, c='black')
limit_x = 1.0
limit_y = 1.0
limit_z = 1.0

ax.set_xlabel("$w_x$")
ax.set_ylabel("$w_y$")
ax.set_zlabel("$w_z$")
ax.grid(True)
ax.set_title('World setting')

 # ======================================  Plot the pixels ==========================================
ax = fig.add_subplot(1, 2, 2)

p20 = pixelCoordsArray[:,0].reshape((2,nPoints), order='F')
ax.plot(p1[0, :],  p1[1, :], 'o', color='red')
ax.plot(p20[0, :], p20[1, :], 'o', color='blue')

ax.set_ylim(0, camera1.height)
ax.set_xlim(0, camera1.width)
ax.grid(True)

ax.legend([mpatches.Patch(color='red'),
         mpatches.Patch(color='blue')],
         ['Desired', 'Init'], loc=2)

ax.plot(pixelCoordsArray[0,0:j], pixelCoordsArray[1,0:j])
ax.plot(pixelCoordsArray[2,0:j], pixelCoordsArray[3,0:j])
ax.plot(pixelCoordsArray[4,0:j], pixelCoordsArray[5,0:j])
ax.plot(pixelCoordsArray[6,0:j], pixelCoordsArray[7,0:j])


fig1 = plt.figure()
ax1 = fig1.add_subplot(2,2,1)
ax1.plot(robot[:,0],robot[:,1],'r-',alpha=0.25)
ax1.plot(trajectory[0,:],trajectory[1,:])
# for center in center_obstacle:
#    circle_robot    = Circle(center, obstacle_raduis , color='r')
#    circle_security = Circle(center, obstacle_raduis + security_margin, color='cyan',fill=False,linestyle='--')
#    ax1.add_patch(circle_robot)
#    #ax1.annotate('Obstacle', xy=center_obstacle, xytext=(1.5, 0.25), arrowprops=dict(facecolor='black', shrink=0.05))
#    ax1.add_patch(circle_security)
ax1.grid()
ax1.set_xlabel("$x$-coordinate [$m$]", fontsize=14)
ax1.set_ylabel("$y$-coordinate [$m$]", fontsize=14)
ax1.set_aspect("equal")

ax2 = fig1.add_subplot(2,2,2)
ax2.plot(times,trajectory[0])
ax2.grid()
ax2.set_xlabel("time [$s$]", fontsize=14)
ax2.set_ylabel("$x$-coordinate [$m$]", fontsize=14)

ax3 = fig1.add_subplot(2,2,3)
ax3.plot(times,trajectory[1])
ax3.grid()
ax3.set_xlabel("time [$s$]", fontsize=14)
ax3.set_ylabel("$y$-coordinate [$m$]", fontsize=14)

ax4 = fig1.add_subplot(2,2,4)
ax4.plot(times,trajectory[2]*(180/np.pi))
ax4.grid()
ax4.set_xlabel("time [$s$]", fontsize=14)
ax4.set_ylabel("Orientation [$deg$]", fontsize=14)

fig2 = plt.figure()

ax5 = fig2.add_subplot(1,2,1)
ax5.plot(times,trajectory[3],label=r"$v \; [\frac{m}{s}$]")
ax5.plot(times,trajectory[4],label=r" $\omega [\frac{rad}{s}$]")
ax5.grid()
ax5.legend()
ax5.set_xlabel(r"time [$s$]", fontsize=14)
ax5.set_ylabel(r"Controls", fontsize=14)

ax6 = fig2.add_subplot(1,2,2)
ax6.plot(times,errors[0],label=r"$e_x \; [m]$")
ax6.plot(times,errors[1],label=r"$e_y \; [m]$")
ax6.plot(times,errors[2],label=r"$e_{yaw} \; [rad]$")
ax6.grid()
ax6.legend()
ax6.set_xlabel(r"time [$s$]", fontsize=14)
ax6.set_ylabel(r"errors", fontsize=14)

plt.show()

print("#========== NonLinear Formulation =============")



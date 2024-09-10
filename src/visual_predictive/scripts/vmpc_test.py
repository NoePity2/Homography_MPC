#=========================================== libraries ======================================================
import numpy as np
import matplotlib.pyplot as plt
import mpc_ddr.utils.Robot as Robot
from mpc_ddr.mpc.nmpc import Pose
import matplotlib.patches as mpatches
from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,H_from_points,Rodrigues,yaw_angle_from_rotationMatrix)
from mpc_ddr.mpc.nmpc import OptimizationHMPC, Homography11,Homography12,Homography13,Homography31,Homography32, Homography33
from mpc_ddr.params.params import orientation_params, position_params, vision_parameters
#from matplotlib import rc
#rc('text', usetex=True)
# plt.rcParams.update({'font.size': 12})
#=========================================== libraries ======================================================

#------------------------------------------------------------------------------------------------------------

#============================================ classes =======================================================
class VMPCPython():

   def __init__(self,params_position=None,params_orientation=None,vision_parameters=None,x_init=0,y_init=0,yaw_init=0,phi_target=0.0,T=0.1,N=15,c_x_r=0.0,c_z_r=0.0,d_value=1.0,n_vector=None,ck_t_ct_vector=None,phi=0.0,He=None):

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
      self.c_x_r      = c_x_r #Traslation between robot with respecto the camera x-axis
      self.c_z_r      = c_z_r

      # Homograpy decomposition
      self.d_value        = d_value
      self.n_vector       = n_vector
      self.ck_t_ct_vector = ck_t_ct_vector
      self.phi            = phi
      self.homo_matrix    = He
      self.PHI_TARGET     = phi_target*np.pi/180
      self.phi_k          = self.PHI_TARGET - yaw_init

      #Velocities (Controls)
      self.angular_velocity = 0.0
      self.V_k              = 0.0

      self.params_position    = params_position
      self.params_orientation = params_orientation
      self.vision_parameters  = vision_parameters

      #States
      self.x_k = np.array( [ [x_init],[y_init],[yaw_init*np.pi/180]] )

      #Homography predicted values
      self.predicted_values = []

      # Solvers
      self.HMPC = OptimizationHMPC(self.params_position,self.params_orientation,self.vision_parameters,self.N,self.T)


   def vel_ops(self):
      # ============ Optimization to obtain linear velocities and angular velocities in a horizon =================
      self.HMPC.solve(self.x_k,self.d_value,self.n_vector,self.ck_t_ct_vector,self.c_x_r,self.c_z_r,self.phi)#False,self.center_obstacle,self.obstacle_raduis+self.security_margin)
      self.angular_velocity = self.HMPC.get_Omega_opt_solution()
      self.V_k = self.HMPC.get_Vopt_solution()
      self.predicted_values = self.HMPC.get_predicted_values()

   def set_homograpy(self,He):
      self.homo_matrix = He

   def set_ck_t_ct_vector(self, ck_t_ct):
      self.ck_t_ct_vector = ck_t_ct

   def set_n_vector(self,n_vector):
      self.n_vector = n_vector

   def set_d_value(self,d_value):
      self.d_value = d_value

   def set_phi_value(self,phi):
      self.phi = phi

   def reset_state(self):
      self.x_k[0] = 0.0
      self.x_k[1] = 0.0
      self.x_k[2] = 0.0

   def update_predictive_values(self):
      self.HMPC.update_predictive_values(self.V_k[0],self.angular_velocity[0],self.x_k,self.T)

   def set_desired_values(self):
      self.HMPC.set_desired_values(self.homo_matrix)

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
   
   def get_h_predictive_values(self):
      return np.array(self.predicted_values)

#============================================ classes ======================================================

#-----------------------------------------------------------------------------------------------------------

#============================================ Parameters ===================================================
N = 15 # number of control intervals

# ============== Linear parameters ==============
state_vector = np.array([[0],[0]])
x  = np.zeros( (3,1) ) 
x_new = np.array( [ [0],[0],[0]] )
params_position = position_params()

dt = 0.1
t0 = 0
tf = 7
times = np.arange(t0,tf,dt)
trajectory = np.zeros(( 5, len(times) )) 

#==================== Visión terms ========================#
#Create the world points
nPoints = 4
xx      = np.array([5.5,5.5,5.5,5.5])
yy      = np.array([-0.5,0.5,0.5,-0.5])
zz      = np.array([0.5,0.5,1.5,1.5])

w_points = np.vstack([xx, yy, zz])

# Set the camera target position
angle_fixed = 90.

# Target position
target_x        = 3.0
target_y        = 0.0
target_z        = 1.0
target_roll     = np.deg2rad(0) # Degrees to radians 'x'
target_pitch    = np.deg2rad(angle_fixed) # Degrees to radians 'y'
target_yaw      = np.deg2rad(0) # Degrees to radians 'z'

camera1 = PlanarCamera() # Set the camera on the target position
camera1.set_position(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
p1 = camera1.projection(w_points, nPoints) # Project the points for camera 1

# Initial position
init_x_pos     = 0.0
init_y_pos     = 0.0
init_z_pos     = 1.0

init_roll    = np.deg2rad(0)
init_pitch   = np.deg2rad(angle_fixed)
init_yaw     = np.deg2rad(0)

camera2 = PlanarCamera() # Set the init camera
camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
p2 = camera2.projection(w_points, nPoints)

pixelCoordsArray  = np.zeros((2*nPoints,len(times)))   # List to save points positions on the image
averageErrorArray = np.zeros(len(times))               # Matrix to save error points positions
z_camera_Array    = np.zeros(len(times)) 
h_predicted_values_array = np.zeros((6,len(times)))
h_values_array = np.zeros( (6,len(times)) )

x_pos = init_x_pos
y_pos = init_y_pos
z_pos = init_z_pos
pitch = init_pitch
yaw   = init_yaw
roll  = init_roll

#=============== Cameras matrix ===============
K1      = camera1.K
K2      = camera2.K
K1_inv  = np.linalg.inv(K1)
K2_inv  = np.linalg.inv(K2)

#============================================ Estimate a first homograpy matriz ==========================
#====================== Estimate the homography matrix ===========================
H = H_from_points(p2,p1)
#====================== Euclidian homograpy matrix ===============================
He = K2_inv.dot(H.dot(K1))
#============== Decomposition of the homography matrix ====================
R,tr,normal,d = homogToRt(He)
#============== Get the yaw angle ===========
yaw_angle_k = -yaw_angle_from_rotationMatrix(R)
#============================================ Parameters ===================================================

#-----------------------------------------------------------------------------------------------------------

#================================================== VMPC ===================================================
params_position    = position_params()
params_orientation = orientation_params()
vision_parameters  = vision_parameters()
PHI_TARGET         = 0.0
Robot_VMPC = VMPCPython(params_position=params_position,params_orientation=params_orientation,vision_parameters=vision_parameters,phi_target=PHI_TARGET,c_z_r=0.0,d_value=d,n_vector=normal,ck_t_ct_vector=tr,phi=-yaw_angle_k,He=He)

#================================================== VMPC ===================================================

#============= Ciclo de navegación ============
for j in range(len(times)):
   # ============ Optimization for angular velocities and linear velocities =================
   Robot_VMPC.vel_ops()
   Omega_opt = Robot_VMPC.angular_velocity[0]
   V_opt     = Robot_VMPC.V_k[0]
   #================================= Update ============================================#
   #%%%%%%%%%%%%%%%%%% UpdatePredictiveValues(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
   Robot_VMPC.update_predictive_values() #Predice los elementos de la homografía
   h_predicted_values = Robot_VMPC.get_h_predictive_values()
   h_predicted_values_array[:, j] = h_predicted_values.reshape((6,))
   #%%%%%%%%%%%%%%%%%% UpdateStates(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
   Robot_VMPC.update_state()
  

   # ========== Apply the first optimal control to a robot ======================
   u = np.array([ V_opt, Omega_opt  ])
   x = Robot.robot_ddr(x_new,u,dt)
   print( "x = ", x )
   state_vector = np.array([  x[0] , x[1]  ]) 
   theta_k = x[2]
   x_new = x

   x_pos = x[0, 0]
   y_pos = x[1, 0]
   pitch = np.deg2rad(angle_fixed) + yaw_angle_k

   camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
   p2 = camera2.projection(w_points, nPoints)
   
   #%%%%%%%%%%%%%%%%%% PrepareDataForNextOptimizacion(opti_solution) %%%%%%%%%%%%%%%%%%%%# 
   
   #================================= Update  visual data ============================================#

   #====================== Estimate the homography matrix ===========================
   H = H_from_points(p2,p1)
   #====================== Euclidian homograpy matrix ===============================
   He = K2_inv.dot(H.dot(K1))
   Robot_VMPC.set_homograpy(He.reshape((9,1)))
   h_values_array[0,j] = He[0,0]
   h_values_array[1,j] = He[0,1]
   h_values_array[2,j] = He[0,2]
   h_values_array[3,j] = He[2,0]
   h_values_array[4,j] = He[2,1]
   h_values_array[5,j] = He[2,2]
   #============== Decomposition of the homography matrix ====================
   R,tr,normal,d = homogToRt(He)
   Robot_VMPC.set_ck_t_ct_vector(tr)
   print("tr: ", tr)
   Robot_VMPC.set_d_value(d)
   Robot_VMPC.set_n_vector(normal)
   #============== Get the yaw angle ===========
   yaw_angle_k = yaw_angle_from_rotationMatrix(R)
   Robot_VMPC.phi_k = Robot_VMPC.PHI_TARGET - yaw_angle_k

   Robot_VMPC.set_desired_values() #valores deseados de los elementos de la homografia
 
   pixelCoordsArray[:,j] = p2.reshape((2*nPoints,1), order='F')[:,0]
   pixel_error           = p2-p1
   averageErrorArray[j]  = np.mean(np.linalg.norm(pixel_error.reshape((2*nPoints,1), order='F')))
   z_camera_Array[j]     = z_pos

   Robot_VMPC.reset_state() # This is due to the optimization is done with respect to the robot's frame

   trajectory[0][j] = x[0]
   trajectory[1][j] = x[1]
   trajectory[2][j] = x[2]
   trajectory[3][j] = u[0]
   trajectory[4][j] = u[1]



robot = Robot.dib_robot([trajectory[0,-1],trajectory[1,-1],trajectory[2,-1]],0.05)

# # ======================================  Draw cameras ========================================

fig = plt.figure()
fig.suptitle('World setting')

ax = fig.add_subplot(2, 2, 1, projection='3d')
# ax = fig.gca(projection='3d')
ax.plot(xx, yy, zz, 'o')

ax.plot(trajectory[0,:],trajectory[1,:],z_camera_Array) # Plot camera trajectory


axis_scale      = 0.5
camera_scale    = 0.09
camera1.draw_camera(ax, scale=camera_scale, color='red')
camera1.draw_frame(ax, scale=axis_scale, c='black')

camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
camera2.draw_camera(ax, scale=camera_scale, color='cyan')
#camera2.draw_frame(ax, scale=axis_scale, c='black')

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
ax = fig.add_subplot(2, 2, 2)

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

ax = fig.add_subplot(2, 2, 3)
colors = ['red','green','blue','magenta','cyan','yellow']
index  = ['h11','h12','h13','h31','h32','h33']
for i in range(6):
   ax.plot(h_predicted_values_array[i, :], color=colors[i], label= r'%s predicted'%index[i],linestyle='--')
   ax.plot(h_values_array[i, :], color=colors[i], label= r'%s observed'%index[i],linestyle='-')

ax.set_xlabel('Time')
ax.set_ylabel('Homography elements')
ax.legend(loc='best', ncol=3)
ax.grid(True)

ax = fig.add_subplot(2, 2, 4)
ax.plot(times,trajectory[3],label=r"$v \; [\frac{m}{s}$]")
ax.plot(times,trajectory[4],label=r" $\omega [\frac{rad}{s}$]")
ax.grid()
ax.legend()
ax.set_xlabel(r"time [$s$]", fontsize=14)
ax.set_ylabel(r"Controls", fontsize=14)


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

# fig2 = plt.figure()

# ax5 = fig2.add_subplot(1,2,1)
# ax5.plot(times,trajectory[3],label=r"$v \; [\frac{m}{s}$]")
# ax5.plot(times,trajectory[4],label=r" $\omega [\frac{rad}{s}$]")
# ax5.grid()
# ax5.legend()
# ax5.set_xlabel(r"time [$s$]", fontsize=14)
# ax5.set_ylabel(r"Controls", fontsize=14)

# ax6 = fig2.add_subplot(1,2,2)
# ax6.plot(times,errors[0],label=r"$e_x \; [m]$")
# ax6.plot(times,errors[1],label=r"$e_y \; [m]$")
# ax6.plot(times,errors[2],label=r"$e_{yaw} \; [rad]$")
# ax6.grid()
# ax6.legend()
# ax6.set_xlabel(r"time [$s$]", fontsize=14)
# ax6.set_ylabel(r"errors", fontsize=14)

plt.show()

# print("#========== NonLinear Formulation =============")



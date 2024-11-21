#=========================================== libraries ======================================================
import numpy as np
import matplotlib.pyplot as plt
import mpc_ddr.utils.Robot as Robot
import matplotlib.patches as mpatches
from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,yaw_angle_from_rotationMatrix,H_from_points_opencv)
from hmpc_utils import VMPCPython, get_3D_target_points, sigma
from mpc_ddr.params.params import orientation_params_simulations, position_params_simulations, vision_parameters_simulations

#from matplotlib import rc
#rc('text', usetex=True)
# plt.rcParams.update({'font.size': 12})
#-----------------------------------------------------------------------------------------------------------



#============================================ Parameters ===================================================
N = 5 # number of control intervals
theta_k_w = 0.0
# ============== Linear parameters ==============
x     = np.array( [ [0],[0],[np.deg2rad(theta_k_w)]] )
x_new = np.array( [ [0],[0],[np.deg2rad(theta_k_w)]] )


dt = 0.1
t0 = 0
tf = 90
times = np.arange(t0,tf,dt)
trajectory = np.zeros(( 5, len(times) ))

#==================== Visión terms ========================#
#Create the world points
n_points = 40
xmin = 12 
ymin = -4.0
ymax = 4.0
zmin = 1
zmax = 6
w_points = get_3D_target_points(n_points,xmin,ymin,ymax,zmin,zmax).transpose()


# Set the camera target position
angle_fixed  = 90.0
theta_target = 0.0
pico_time = 0

# Target position of camera 1
target_x     = 3.0
target_y     = 0
target_z     = 1.0
target_roll  = np.deg2rad(0) # Degrees to radians 'x'
target_pitch = np.deg2rad(angle_fixed - theta_target) # Degrees to radians 'y'
target_yaw   = np.deg2rad(0) # Degrees to radians 'z'

# Target position of camera 2 
target_x2     = 6.0
target_y2     = 0.0
target_z2     = 1.0
target_roll2  = np.deg2rad(0) # Degrees to radians 'x'
target_pitch2 = np.deg2rad(angle_fixed- theta_target) # Degrees to radians 'y'
target_yaw2   = np.deg2rad(0) # Degrees to radians 'z'

# Target position of camera 3 
target_x3     = 10.0
target_y3     = 0.0
target_z3     = 1.0
target_roll3  = np.deg2rad(0) # Degrees to radians 'x'
target_pitch3 = np.deg2rad(angle_fixed- theta_target) # Degrees to radians 'y'
target_yaw3   = np.deg2rad(0) # Degrees to radians 'z'

#Calibration matrix  
K = np.array([[522.1910329546544,0.0,320.5],
               [0.0, 522.1910329546544,240.5],
               [0.,0.,1.]])

#Set the first camera on target position.
camera1 = PlanarCamera() 
camera1.set_K(K)
camera1.set_position(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
p1 = camera1.projection(w_points, n_points) # Project the points for camera 1

#Set the second camera on target position.
camera2 = PlanarCamera() 
camera2.set_K(K)
camera2.set_position(target_x2, target_y2, target_z2, target_roll2, target_pitch2, target_yaw2)
p2 = camera2.projection(w_points, n_points) # Project the points for camera 2

#Set the second camera on target position.
camera3 = PlanarCamera() 
camera3.set_K(K)
camera3.set_position(target_x3, target_y3, target_z3, target_roll3, target_pitch3, target_yaw3)
p3 = camera3.projection(w_points, n_points) # Project the points for camera 2

# Initial position
init_x_pos = 0.0
init_y_pos = 0.0
init_z_pos = 1.0

init_roll  = np.deg2rad(0)
init_pitch = np.deg2rad(angle_fixed - theta_k_w)
init_yaw   = np.deg2rad(0)

# Set the init camera
camera = PlanarCamera() 
camera.set_K(K)
camera.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
camera.set_noise(0)
p_k = camera.projection(w_points, n_points)

pixelCoordsArray         = np.zeros((2*n_points,len(times)))   # List to save points positions on the image
averageErrorArray        = np.zeros(len(times))               # Matrix to save error points positions
z_camera_Array           = np.zeros(len(times)) 
h_values_array           = np.zeros( (6,len(times)) )
h_predicted_values_array = np.zeros((6,len(times)))

x_pos = init_x_pos
y_pos = init_y_pos
z_pos = init_z_pos
pitch = init_pitch
yaw   = init_yaw
roll  = init_roll

#=============== Cameras matrix ===============
K      = camera1.K
K_inv  = np.linalg.inv(K)

#============================================ Estimate a first homograpy matriz ==========================
#====================== Estimate the homography matrix ===========================
#H = H_from_points(p2,p1)
H = H_from_points_opencv(p_k,p1)
#====================== Euclidian homograpy matrix ===============================
He = K_inv.dot(H.dot(K))
#============== Decomposition of the homography matrix ====================
R,tr,normal,d = homogToRt(He)
print(normal)
#============== Get the yaw angle ===========
yaw_angle_k = -yaw_angle_from_rotationMatrix(R)
#============================================ Parameters ===================================================

#================================================== VMPC ===================================================
params_position    = position_params_simulations()
params_orientation = orientation_params_simulations()
vision_parameters  = vision_parameters_simulations()
PHI_TARGET         = 0.0
Robot_VMPC = VMPCPython(params_position=params_position,params_orientation=params_orientation,vision_parameters=vision_parameters,phi_target=PHI_TARGET,c_z_r=0.0,d_value=d,n_vector=normal,ck_t_ct_vector=tr,phi=-yaw_angle_k,He=He)


def sat_smooth_exp(x, L_min, M):
    return L_min + (M - L_min) * np.exp(-x)

M = 0.26
#================================================== VMPC ===================================================
p_t = p1
task = 1

#============= Simulation ============
for j in range(len(times)):

   # ============ Optimization for angular velocities and linear velocities =================
   Robot_VMPC.vel_ops()
   Omega_opt = Robot_VMPC.angular_velocity[0]
   V_opt     = Robot_VMPC.V_k[0]

   #================================= Update ============================================#

   #%%%%%%%%%%%%%%%%%% Update Predicted Values(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
   Robot_VMPC.update_predictive_values() #Predice los elementos de la homografía
   h_predicted_values = Robot_VMPC.get_h_predictive_values()
   h_predicted_values_array[:, j] = h_predicted_values.reshape((6,))
   #%%%%%%%%%%%%%%%%%% Update States(opti_solution) %%%%%%%%%%%%%%%%%%%%%%%%%%%%#
   Robot_VMPC.update_state()
  
   # ========== Apply the first optimal control to a robot ======================
   u = np.array([ V_opt, Omega_opt  ])

   #Saturation function
   if (task < 3):
      #u[0] = sigma(u[0], L, M)
      u[0] = sat_smooth_exp(u[0], 0.259, M)
      #u[1] = sat_smooth_exp(u[1], 0.7, M) 
   else:
      if (pico_time > 2):
         u[0] = sigma(u[0]*2, 0.259, M)
      else:
         u[0] = M
         pico_time = pico_time + 1
         

   x = Robot.robot_ddr(x_new,u,dt)
   print( "x = ", x )
   theta_k_w = x[2]
   x_new = x

   x_pos = x[0, 0]
   y_pos = x[1, 0]
   pitch = np.deg2rad(angle_fixed) - theta_k_w
   print("theta_w: ", theta_k_w*180/np.pi)

   #==================== Update the camera position ===========================
   camera.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
   p_k = camera.projection(w_points, n_points)

   #%%%%%%%%%%%%%%%%%% Prepare Data For Next Optimizacion %%%%%%%%%%%%%%%%%%%%# 
   
   #================================= Update  visual data ============================================#

   #====================== Estimate the homography matrix ===========================
   H = H_from_points_opencv(p_k,p_t)

   #====================== Euclidian homograpy matrix ===============================
   He = K_inv.dot(H.dot(K))
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
   Robot_VMPC.set_d_value(d)
   Robot_VMPC.set_n_vector(normal)
   #============== Get the yaw angle ===========
   phi_k = yaw_angle_from_rotationMatrix(R)
   Robot_VMPC.set_phi_value( phi_k)
   Robot_VMPC.set_desired_values() #valores deseados de los elementos de la homografia
 
   pixelCoordsArray[:,j] = p_k.reshape((2*n_points,1), order='F')[:,0]
   pixel_error           = p_k-p_t

   #averageErrorArray[j]  = np.mean(np.linalg.norm(pixel_error.reshape((2*n_points,1), order='F')))
   averageErrorArray[j] = np.mean(np.linalg.norm(pixel_error, axis=0))
   if averageErrorArray[j] < 5 and task < 3:
      task += 1
      if task == 2:
         p_t = p2
      elif task == 3:
         p_t = p3
      

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
ax.plot(w_points[0,:], w_points[1,:], w_points[2,:], 'o')
ax.plot(trajectory[0,:],trajectory[1,:],z_camera_Array) # Plot camera trajectory

axis_scale      = 0.5
camera_scale    = 0.09
camera1.draw_camera(ax, scale=camera_scale, color='red')
#camera1.draw_frame(ax, scale=axis_scale, c='black')

camera2.draw_camera(ax, scale=camera_scale, color='orange')

camera3.draw_camera(ax, scale=camera_scale, color='green')
#camera1.draw_frame(ax, scale=axis_scale, c='black')

## camara azul clarito posicion a la que llego
camera.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
camera.draw_camera(ax, scale=camera_scale, color='cyan')
#camera2.draw_frame(ax, scale=axis_scale, c='black')

## camara azul posicion inicial
camera.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
camera.draw_camera(ax, scale=camera_scale, color='blue')
#camera2.draw_frame(ax, scale=axis_scale, c='black')
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

p20 = pixelCoordsArray[:,0].reshape((2,n_points), order='F')
ax.plot(p1[0, :],  p1[1, :], 'o', color='red')
ax.plot(p20[0, :], p20[1, :], 'o', color='blue')

# ax.set_ylim(0, camera1.height)
# ax.set_xlim(0, camera1.width)
ax.grid(True)

ax.legend([mpatches.Patch(color='red'),
         mpatches.Patch(color='blue')],
         ['Desired', 'Init'], loc=2)

for i in range(n_points):
   ax.plot(pixelCoordsArray[2*i,0:j], pixelCoordsArray[2*i+1,0:j],color='green',linewidth=0.5) 

#### homografia
ax = fig.add_subplot(2, 2, 3)
colors = ['red','green','blue','magenta','cyan','yellow']
index  = ['h11','h12','h13','h31','h32','h33']
# Graficar las líneas
for i in range(6):
    ax.plot(times, h_predicted_values_array[i, :], color=colors[i], label=f'{index[i]} predicted', linestyle='--')
    ax.plot(times, h_values_array[i, :], color=colors[i], label=f'{index[i]} observed', linestyle='-')

ax.set_xlabel('Time')
ax.set_ylabel('Homography elements')
ax.legend(loc='best', ncol=3)
ax.grid(True)


##### Control Velocities
ax = fig.add_subplot(2, 2, 4)
ax.plot(times,trajectory[3],label=r"$v \; [\frac{m}{s}$]", color = 'orange')
ax.plot(times,trajectory[4],label=r" $\omega [\frac{rad}{s}$]", color = 'blue')
ax.grid()
ax.legend()
ax.set_xlabel(r"time [$s$]", fontsize=14)
ax.set_ylabel(r"Controls", fontsize=14)

####================= Figura 2 ==========================#### 

# fig1 = plt.figure()
# ax1 = fig1.add_subplot(2,2,1)
# ax1.plot(robot[:,0],robot[:,1],'r-',alpha=0.25)
# ax1.plot(trajectory[0,:],trajectory[1,:])
# ax1.grid()
# ax1.set_xlabel("$x$-coordinate [$m$]", fontsize=14)
# ax1.set_ylabel("$y$-coordinate [$m$]", fontsize=14)
# ax1.set_aspect("equal")

# ax2 = fig1.add_subplot(2,2,2)
# ax2.plot(times,trajectory[0])
# ax2.grid()
# ax2.set_xlabel("time [$s$]", fontsize=14)
# ax2.set_ylabel("$x$-coordinate [$m$]", fontsize=14)

# ax3 = fig1.add_subplot(2,2,3)
# ax3.plot(times,trajectory[1])
# ax3.grid()
# ax3.set_xlabel("time [$s$]", fontsize=14)
# ax3.set_ylabel("$y$-coordinate [$m$]", fontsize=14)

# ax4 = fig1.add_subplot(2,2,4)
# ax4.plot(times,trajectory[2]*(180/np.pi))
# ax4.grid()
# ax4.set_xlabel("time [$s$]", fontsize=14)
# ax4.set_ylabel("Orientation [$deg$]", fontsize=14)

plt.show()


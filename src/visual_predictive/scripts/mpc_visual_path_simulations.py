#=========================================== libraries ======================================================
import numpy as np
import matplotlib.pyplot as plt
import mpc_ddr.utils.Robot as Robot
import matplotlib.patches as mpatches
from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,H_from_points,yaw_angle_from_rotationMatrix,H_from_points_opencv)
from mpc_ddr.mpc.nmpc import OptimizationHMPC
from mpc_ddr.params.params import orientation_params_simulations, position_params_simulations, vision_parameters_simulations

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
      #self.PHI_TARGET     = phi_target*np.pi/180


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

def get_3D_target_points(n_points,xmin,ymin,ymax,zmin,zmax):

    w_points = []
    for _ in range(n_points):
        xx = np.random.uniform(xmin, xmin)
        yy = np.random.uniform(ymin, ymax)
        zz = np.random.uniform(zmin, zmax)
        w_points.append([xx, yy, zz])

    return np.asarray(w_points)


#============================================ classes ======================================================

#-----------------------------------------------------------------------------------------------------------

#============================================= Logica de Función de saturación =========================================
def sigma(s, L, M):
    # Calcular la constante c
    c = np.pi / (2 * (M - L))
    
    # Implementar la función de saturación suave para ambos límites
    if s > L:
        return (1/c) * np.tanh(c * (s - L)) + L
    elif s < -L:
        return -(1/c) * np.tanh(c * (-s - L)) - L
    else:
        return s
     
# Definición de parámetros
L = 0.0  # Valor de L
M = 0.2  # Valor de M
#============================================= Logica de Función de saturación =========================================

#============================================ Parameters ===================================================
N = 5 # number of control intervals
theta_k_w = 0.0
# ============== Linear parameters ==============
x     = np.array( [ [0],[0],[np.deg2rad(theta_k_w)]] )
x_new = np.array( [ [0],[0],[np.deg2rad(theta_k_w)]] )


dt = 0.1
t0 = 0
tf = 150
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

xmin = 15 
w_points_2 = get_3D_target_points(n_points,xmin,ymin,ymax,zmin,zmax).transpose()

# Set the camera target position
angle_fixed  = 90.0
theta_target = 30.0

# Target position de la camara 1 (imagen deseada 1)
target_x     = 4.0
target_y     = 2
target_z     = 1.0
target_roll  = np.deg2rad(0) # Degrees to radians 'x'
target_pitch = np.deg2rad(angle_fixed - theta_target) # Degrees to radians 'y'
target_yaw   = np.deg2rad(0) # Degrees to radians 'z'

# Target position de la camara 3 (imagen deseada 2)
target_x3     = 8.0
target_y3     = 2
target_z3     = 1.0
target_roll3  = np.deg2rad(0) # Degrees to radians 'x'
target_pitch3 = np.deg2rad(angle_fixed) # Degrees to radians 'y'
target_yaw3   = np.deg2rad(0) # Degrees to radians 'z'

#Calibration matrix
K = np.array([[522.1910329546544,0.0,320.5],
               [0.0, 522.1910329546544,240.5],
               [0.,0.,1.]])

#camara roja primer objetivo
camera1 = PlanarCamera() # Set the camera on the target position
camera1.set_K(K)
camera1.set_position(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
p1 = camera1.projection(w_points, n_points) # Project the points for camera 1

#camara verde segundo objetivo
camera3 = PlanarCamera() # Set the camera on the target position
camera3.set_K(K)
camera3.set_position(target_x3, target_y3, target_z3, target_roll3, target_pitch3, target_yaw3)
#p3 = camera3.projection(w_points_2, n_points) # Project the points for camera 1
p3 = camera3.projection(w_points, n_points) # Project the points for camera 3

# Initial position
init_x_pos = 0.0
init_y_pos = 0.0
init_z_pos = 1.0

init_roll  = np.deg2rad(0)
init_pitch = np.deg2rad(angle_fixed - theta_k_w)
init_yaw   = np.deg2rad(0)

## camara imagenes actuales (azules)
camera2 = PlanarCamera() # Set the init camera
camera2.set_K(K)
camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
camera2.set_noise(0)
p2 = camera2.projection(w_points, n_points)

pixelCoordsArray         = np.zeros((2*n_points,len(times)))   # List to save points positions on the image
averageErrorArray        = np.zeros(len(times))               # Matrix to save error points positions
z_camera_Array           = np.zeros(len(times)) 
h_values_array           = np.zeros( (6,len(times)) )
h_values_array_2           = np.zeros( (6,len(times)) )
h_predicted_values_array = np.zeros((6,len(times)))
h_predicted_values_array_2 = np.zeros((6,len(times)))

x_pos = init_x_pos
y_pos = init_y_pos
z_pos = init_z_pos
pitch = init_pitch
yaw   = init_yaw
roll  = init_roll

#=============== Cameras matrix ===============
K1      = camera1.K
K2      = camera2.K
K3      = camera3.K ##borrar posible
print(K2)
K1_inv  = np.linalg.inv(K1)
K2_inv  = np.linalg.inv(K2)
K3_inv  = np.linalg.inv(K3) ##borrar posible


#============================================ Estimate a first homograpy matriz ==========================
#====================== Estimate the homography matrix ===========================
#H = H_from_points(p2,p1)
H = H_from_points_opencv(p2,p1)
#====================== Euclidian homograpy matrix ===============================
He = K2_inv.dot(H.dot(K1))
#============== Decomposition of the homography matrix ====================
R,tr,normal,d = homogToRt(He)
print(normal)
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

points = p1
matriz_calib = K1
task = 1
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
   print("Vector u:")
   print(u) 
   if (task == 2):
      u = np.array([sigma(s, L, M) for s in u]) # aqui meto la función de saturación
   x = Robot.robot_ddr(x_new,u,dt)
   print( "x = ", x )
   theta_k_w = x[2]
   x_new = x

   x_pos = x[0, 0]
   y_pos = x[1, 0]
   pitch = np.deg2rad(angle_fixed) - theta_k_w
   print("theta_w: ", theta_k_w*180/np.pi)

   camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
   p2 = camera2.projection(w_points, n_points)

   #%%%%%%%%%%%%%%%%%% PrepareDataForNextOptimizacion(opti_solution) %%%%%%%%%%%%%%%%%%%%# 
   
   #================================= Update  visual data ============================================#

   #====================== Estimate the homography matrix ===========================
   H = H_from_points_opencv(p2,points)
   print("Matriz H:")
   print(H) 
   #====================== Euclidian homograpy matrix ===============================
   He = K2_inv.dot(H.dot(matriz_calib))
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
   print("rotation diference: ", tr)
   Robot_VMPC.set_phi_value( phi_k)

   Robot_VMPC.set_desired_values() #valores deseados de los elementos de la homografia
 
   pixelCoordsArray[:,j] = p2.reshape((2*n_points,1), order='F')[:,0]
   pixel_error           = p2-points
   #print("pixel_error")
   #print(pixel_error) 
   #averageErrorArray[j]  = np.mean(np.linalg.norm(pixel_error.reshape((2*n_points,1), order='F')))
   averageErrorArray[j] = np.mean(np.linalg.norm(pixel_error, axis=0))
   if averageErrorArray[j] < 18 and task == 1:
      points = p3
      matriz_calib = K3
      task = 2
   #print("Average Error Array")
   #print(averageErrorArray) 
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

camera3.draw_camera(ax, scale=camera_scale, color='orange')
#camera1.draw_frame(ax, scale=axis_scale, c='black')

## camara azul clarito posicion a la que llego
camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
camera2.draw_camera(ax, scale=camera_scale, color='cyan')
#camera2.draw_frame(ax, scale=axis_scale, c='black')

## camara azul posicion inicial
camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
camera2.draw_camera(ax, scale=camera_scale, color='blue')
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

plt.show()

#####====================== Figura 3 =======================######
# Crear una nueva figura para la homografía
fig_homography = plt.figure(3)
ax5 = fig_homography.add_subplot(1, 1, 1)  # Un único subplot en la figura

# Definir colores e índices
colors = ['red', 'green', 'blue', 'magenta', 'cyan', 'yellow']
index = ['h11', 'h12', 'h13', 'h31', 'h32', 'h33']

# Graficar las líneas
for i in range(6):
    ax5.plot(times, h_predicted_values_array[i, :], color=colors[i], label=f'{index[i]} predicted', linestyle='--')
    ax5.plot(times, h_values_array[i, :], color=colors[i], label=f'{index[i]} observed', linestyle='-')

# Configuración de los ejes y la leyenda
ax5.set_xlabel('Time')
ax5.set_ylabel('Homography elements')
ax5.legend(loc='best', ncol=3)
ax5.grid(True)

# Mostrar la figura
plt.show()



# print("#========== NonLinear Formulation =============")

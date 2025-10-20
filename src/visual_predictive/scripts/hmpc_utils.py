import numpy as np
from mpc_ddr.mpc.nmpc import OptimizationHMPC




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
   

# This function gives a set of points in the world.
def get_3D_target_points(n_points,xmin,ymin,ymax,zmin,zmax):

    w_points = []
    for _ in range(n_points):
        xx = np.random.uniform(xmin, xmin)
        yy = np.random.uniform(ymin, ymax)
        zz = np.random.uniform(zmin, zmax)
        w_points.append([xx, yy, zz])

    return np.asarray(w_points)


# This is a saturation function.
def sigma(x, L, M):

    c = np.pi / (2 * (M - L))
    
    if x > L:
        return x
        
    else:
        return L + (np.tanh(c*(x-L)))/c


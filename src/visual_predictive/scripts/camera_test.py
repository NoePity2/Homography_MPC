import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,H_from_points,Rodrigues,yaw_angle_from_rotationMatrix)

from matplotlib import rc

rc('text', usetex=True)
# plt.rcParams.update({'font.size': 12})




#==================== Visión terms ========================#
#Create the world points
n_points = 20

xmin = 5.5 
ymin = 1.0
ymax = 4.0
zmin = 1
zmax = 6
w_points = get_3D_target_points(n_points,xmin,ymin,ymax,zmin,zmax).transpose()

print(w_points)

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
p1 = camera1.projection(w_points, n_points) # Project the points for camera 1

# Initial position
init_x_pos     = 0.0
init_y_pos     = 0.0
init_z_pos     = 1.0

init_roll    = np.deg2rad(0.0)
init_pitch   = np.deg2rad(angle_fixed - 10.0)
init_yaw     = np.deg2rad(0.0)

camera2 = PlanarCamera() # Set the init camera
camera2.set_position(init_x_pos, init_y_pos, init_z_pos, init_roll, init_pitch, init_yaw)
p2 = camera2.projection(w_points, n_points) # Project the points for camera 2

# #=============== Cameras matrix ===============
# K1      = camera1.K
# K2      = camera2.K
# K1_inv  = np.linalg.inv(K1)
# K2_inv  = np.linalg.inv(K2)

# #============================================ Estimate a first homograpy matriz ==========================
# #====================== Estimate the homography matrix ===========================
# H = H_from_points(p2,p1)
# #====================== Euclidian homograpy matrix ===============================
# He = K2_inv.dot(H.dot(K1))
# #============== Decomposition of the homography matrix ====================
# R,tr,normal,d = homogToRt(He)
# print("tr: ", tr)
# #============== Get the yaw angle ===========
# yaw_angle_k = yaw_angle_from_rotationMatrix(R)

# ======================================  Draw cameras ========================================

fig = plt.figure()
fig.suptitle('World setting')

ax = fig.add_subplot(1, 1, 1, projection='3d')
#ax = fig.gca(projection='3d')
ax.plot(w_points[0,:], w_points[1,:], w_points[2,:], 'o')

axis_scale      = 0.5
camera_scale    = 0.09
camera1.draw_camera(ax, scale=camera_scale, color='red')
#camera1.draw_frame(ax, scale=axis_scale, c='black')


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
fig = plt.figure()
ax = fig.add_subplot(1, 2, 2)

ax.plot(p1[0, :],  p1[1, :], 'o', color='red')
ax.plot(p2[0, :], p2[1, :], 'o', color='blue')

ax.set_ylim(0, camera1.height)
ax.set_xlim(0, camera1.width)
ax.grid(True)

ax.legend([mpatches.Patch(color='red'),
         mpatches.Patch(color='blue')],
         ['Desired', 'Init'], loc=2)

plt.show()

print("#========== NonLinear Formulation =============")



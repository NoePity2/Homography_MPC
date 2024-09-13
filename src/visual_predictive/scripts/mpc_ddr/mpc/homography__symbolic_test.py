import casadi as opt
import numpy as np

rk_x_rl     = opt.SX.sym('rk_x_rl')
rk_y_rl     = opt.SX.sym('rk_y_rl')
rk_theta_rl = opt.SX.sym('rk_theta_rl')

ck_t_ct = opt.SX.sym('ck_t_ct',3)
n       = opt.SX.sym('n',3)
phi_k   = opt.SX.sym('phi_k')
c_x_r   = opt.SX.sym('c_x_r')
c_z_r   = opt.SX.sym('c_z_r')
d       = opt.SX.sym('d')
cl_t_ct = opt.SX.sym('cl_t_ct',3)

cl_t_ct[0] = (ck_t_ct[0] + rk_y_rl )*opt.cos(rk_theta_rl) + (ck_t_ct[2] + rk_x_rl )*opt.sin(rk_theta_rl) 
cl_t_ct[1] = 0.0
cl_t_ct[2] = (-ck_t_ct[0] + rk_y_rl )*opt.sin(rk_theta_rl) + (ck_t_ct[2] - rk_x_rl )*opt.cos(rk_theta_rl) 

# cl_t_ct[0] = (ck_t_ct[0] + rk_y_rl - c_x_r)*opt.cos(rk_theta_rl) + (ck_t_ct[2] + rk_x_rl - c_z_r)*opt.sin(rk_theta_rl) + c_x_r
# cl_t_ct[1] = 0.0
# cl_t_ct[2] = (-ck_t_ct[0] + rk_y_rl + c_x_r)*opt.sin(rk_theta_rl) + (ck_t_ct[2] - rk_x_rl - c_z_r)*opt.cos(rk_theta_rl) + c_z_r

I = opt.SX.eye(3)   
cl_R_ct = opt.SX.sym('cl_R_ct',3,3) 
cl_R_ct[0,0] = opt.cos(phi_k + rk_theta_rl)
cl_R_ct[0,1] = 0.0
cl_R_ct[0,2] = opt.sin(phi_k + rk_theta_rl) 
cl_R_ct[1,0] = 0.0
cl_R_ct[1,1] = 1.0
cl_R_ct[1,2] = 0.0
cl_R_ct[2,0] = -opt.sin(phi_k + rk_theta_rl)
cl_R_ct[2,1] = 0.0
cl_R_ct[2,2] = opt.cos(phi_k + rk_theta_rl)

t_H_l = cl_R_ct.T @ (I - cl_t_ct@(n.T/d)) 
print(t_H_l[0,0])
import numpy as np


def dib_robot(location,side=0.4):
	"""
	dib_robot(location,attributes,lado,color)

	@param location [x y chi]
	@param side (optativo) tamano del robot

	@return Una lista con el modelo robot: 
              |x
              |
        b-----e-----c
        |    /|\    |
        |   / | \   |
  y ----------0  \  |
        | /       \ |
        |/         \|
        a-----------d

	"""
	side_coseno = side*np.cos(location[2])
	side_seno   = side*np.sin(location[2])
	model = np.array([
		              [-1,  1, 1],#a
					  [ 1,  1, 1],
					  [ 1, -1, 1],
					  [-1, -1, 1],
					  [-1,  1, 1],
					  [ 1,  0, 1],
					  [-1, -1, 1] 
					  ])

	transformation = np.array([
								[side_coseno, -side_seno  , location[0]],
								[side_seno  ,  side_coseno, location[1]],
								[     0     ,       0     ,      1]
							 ])

	robot = np.dot( transformation,model.transpose() )
	robot = robot.transpose()

	return robot


def robot_ddr(x,u,dt):
    """
    Esta función representa el modelo cinemático de un DDR.

    @param x vector de estado (x_r,y_r,theta_r)
    @param u vector de controles (v_r,w_r)
    @param dt Tiempo de muestreo
 
    @return x_new El nuevo vector de estado obtenido al aplicar los controles U
    """
    x_new   = np.zeros( (3,1) )
    vr_k    = u[0]
    omega_k = u[1]
    theta_k = x[2]

    #============================ System model ============================
    x_new[0] = x[0] + dt*vr_k*np.cos(theta_k) # x(k+1) = x_k + dt*v_k*cos(theta_k)
    x_new[1] = x[1] + dt*vr_k*np.sin(theta_k) # y(k+1) = y_k + dt*v_k*sin(theta_k)
    x_new[2] = x[2] + dt*omega_k              # theta(k+1) = theta_k + dt*omega_k
    
    return x_new
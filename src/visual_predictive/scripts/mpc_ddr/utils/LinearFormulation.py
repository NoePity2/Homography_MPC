import numpy as np


#--------------------------Funciones--------------------------#

def Ppst(N):
    ma = np.ones((N,1))
    
    return ma

def Pput(N,T): 
    
    ma = np.tri(N, N, 0)

    """

    [ T 0 0 0 ... ]
    [ T T 0 0 ... ]
    [ T T T 0 ... ]
    [ T T T T ... ]
    [ . . . . . . ]

    """
    return T*ma

def Ppsy(N=10):
    """
    Input : N Horizon
    Salida: Ppsy 
    """
    zeros = np.zeros( (N,1) )
    ones  = np.ones( (N,1) )
    mat   = np.hstack( (zeros,ones) )

    return mat

def Ppsx(N):
    """
    Input : N Horizon
    Salida: Ppsy 
    """
    zeros = np.zeros( (N,1) )
    ones  = np.ones( (N,1) )
    mat   = np.hstack( (ones,zeros) )

    return mat

def Ppuy(N=10,T=0.1,tethas_vector=None):
    """
    Inputs: 
      N Horizon
      T Discretization time 
      
    Output: Ppuy
    [ T*sin( tetha[k] )        0.0          ...           0.0         ]
    [ T*sin( tetha[k] ) T*sin( tetha[k+1] ) ...           0.0         ]
    [ T*sin( tetha[k] ) T*sin( tetha[k+1] ) ...  T*sin( tetha[k+N-1]) ]
   
    """
    Ppu = np.zeros((N,N))
        
    for j in range(0,N):
        for i in range(0,j+1):
            Ppu[j][i]= T*np.sin(tethas_vector[i])
                    
    return Ppu
    

def Ppux(N=10,T=0.1,tethas_vector=None): 
    """
    Inputs: 
      N Horizon
      T Discretization time 
      
    Output: Ppuy
    [ T*cos( tetha[k] )        0.0          ...           0.0         ]
    [ T*cos( tetha[k] ) T*cos( tetha[k+1] ) ...           0.0         ]
    [ T*cos( tetha[k] ) T*cos( tetha[k+1] ) ...  T*cos( tetha[k+N-1]) ]
   
    """

    Ppu = np.zeros((N,N))

    for j in range(0,N):
        for i in range(0,j+1):
            Ppu[j][i]= T*np.cos(tethas_vector[i])
                    
    return Ppu

#===================== Pose optimization =================================
def Get_Q_pose(Ppux_m,Ppuy_m,In,alpha1,alpha2,alpha3):
    return alpha1*In + alpha2 * np.dot(Ppux_m.transpose(), Ppux_m) + alpha3 * np.dot(Ppuy_m.transpose(), Ppuy_m)

def Get_p_pose(Ppux_m,Ppuy_m,Ppsx_m,Ppsy_m,x_k,X_ref,Y_ref,alpha2,alpha3):
    return alpha2 * np.dot(Ppux_m.transpose(), (np.dot(Ppsx_m, x_k) - X_ref)) + alpha3 * np.dot(Ppuy_m.transpose(), ( np.dot(Ppsy_m, x_k) - Y_ref))


#====================== Prediction vectors =================================
def Get_X_k1_vector(Ppsx,Ppux,x_k,V_k):
	return np.dot(Ppsx,x_k) + np.dot(Ppux,V_k)

def Get_Y_k1_vector(Ppsy,Ppuy,x_k,V_k):
	return np.dot(Ppsy,x_k) + np.dot(Ppuy,V_k)

def Get_angles_from_optimization(Ppst_m,theta_k, Pput_m,angular_velocity):
    return Ppst_m*theta_k + np.dot(Pput_m,angular_velocity)



#========================= Orientation OPtimization ========================
def Get_Q_orientation(Pput_m,In,alpha,beta):
	Q_k = alpha*In + beta * np.dot(Pput_m.transpose(), Pput_m)
	return Q_k

def Get_p_orientation(Pput_m,Ppst_m,beta,theta_k,Tethas_ref):
	p_k = beta * np.dot(Pput_m.transpose(), ( Ppst_m*theta_k - Tethas_ref)) 
	return p_k

#================== Gx <= h constraints ==============0
def Get_G_orientation(Pput_m):
	#=== https://stackoverflow.com/questions/32543475/how-python-cvxopt-solvers-qp-basically-works ===
	#======= Transpose due to the cvxopt syntax =======================
	G_k = np.vstack((Pput_m.transpose(),-Pput_m.transpose()))
	return G_k

def Get_h_orientation(N,Ppst_m,theta_k,Thetas_OMN,DELTA):
	p1 = np.ones((N,1))*DELTA + Thetas_OMN - Ppst_m*theta_k
	p2 = np.ones((N,1))*DELTA - Thetas_OMN + Ppst_m*theta_k
	h  = np.vstack((p1,p2))
	return h

def Get_Thetas_OMN(X,Y):
	return np.arctan2(Y, X)

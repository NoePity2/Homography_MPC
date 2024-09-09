"""
Parameters for lineal mpc
"""

__author__ = 'Noe Aldana'
__email__ = 'noe.aldana@iberoleon.mx'

import numpy as np

def orientation_params():

	delta_min = 0.0  # angle limit
	delta_max = 5*(np.pi/180) # angle limit
	alpha = 1e-2 # qp gain
	beta  = 1e6 # 2e-2 non linear 
	lbO   = -0.6 # lower angular velocity constraint
	ubO   = 0.6 # upper angular velocity constraint

	params = {
		'delta_min': delta_min,
		'delta_max': delta_max,
		'alpha_w': alpha,
		'beta' : beta,
		'lbO'  : lbO,
		'ubO'  : ubO,
	}

	return params

def position_params():

	alpha1 = 1e-3
	lbv    = -0.26 # lower velocity constraint
	ubv    = 0.26 # upper velocity constraint

	params = {
		'alpha_v': alpha1,
		'lbv'   : lbv,
		'ubv'   : ubv,
	}

	return params

def vision_parameters():

	b11 = 7000
	b12 = 7000
	b13 = 7000
	b31 = 7000
	b32 = 7000
	b33 = 7000

	params = {
		'b11': b11,
		'b12': b12,
		'b13': b13,
		'b31': b31,
		'b32': b32,
		'b33': b33,
	}

	return params



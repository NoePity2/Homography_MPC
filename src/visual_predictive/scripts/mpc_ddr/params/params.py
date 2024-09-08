"""
Parameters for lineal mpc
"""

__author__ = 'Noe Aldana'
__email__ = 'noe.aldana@iberoleon.mx'

import numpy as np

def orientation_params():

	delta_min = 0.0  # angle limit
	delta_max = 2*np.pi
	alpha = 1e-2 # qp gain
	beta  = 1e2 # 2e-2 non linear 
	lbO   = -0.3 # lower angular velocity constraint
	ubO   = 0.3 # upper angular velocity constraint

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

	alpha1 = 1e-2
	alpha2 = 1.2e-1
	alpha3 = 1.2e-1
	lbv    = -0.2 # lower velocity constraint
	ubv    = 0.2 # upper velocity constraint

	params = {
		'alpha_v': alpha1,
		'alpha2': alpha2,
		'alpha3': alpha3,
		'lbv'   : lbv,
		'ubv'   : ubv,
	}

	return params

def vision_parameters():

	b11 = 1e1
	b12 = 1e1
	b13 = 1e1
	b31 = 1e0
	b32 = 1e0
	b33 = 2

	params = {
		'b11': b11,
		'b12': b12,
		'b13': b13,
		'b31': b31,
		'b32': b32,
		'b33': b33,
	}

	return params



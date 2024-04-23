"""
Parameters for lineal mpc
"""

__author__ = 'Noe Aldana'
__email__ = 'noe.aldana@iberoleon.mx'

import numpy as np

def orientation_params():

	delta_min = 0.0  # angle limit
	delta_max = 2*np.pi
	alpha = 1e-1 # qp gain
	beta  = 2e-2 # 2e-2 non linear 
	lbO   = -1.81829 # lower angular velocity constraint
	ubO   = 1.81829 # upper angular velocity constraint

	params = {
		'delta_min': delta_min,
		'delta_max': delta_max,
		'alpha': alpha,
		'beta' : beta,
		'lbO'  : lbO,
		'ubO'  : ubO,
	}

	return params

def position_params():

	alpha1 = 0.1
	alpha2 = 1.2e-1
	alpha3 = 1.2e-1
	lbv    = -0.2 # lower velocity constraint
	ubv    = 0.2 # upper velocity constraint

	params = {
		'alpha1': alpha1,
		'alpha2': alpha2,
		'alpha3': alpha3,
		'lbv'   : lbv,
		'ubv'   : ubv,
	}

	return params





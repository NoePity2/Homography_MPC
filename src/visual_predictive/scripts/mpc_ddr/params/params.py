"""
Parameters for lineal mpc
"""


import numpy as np

def orientation_params():

	alpha = 4e2# qp gain
	beta  = 1.5e-1 # non linear
	lbO   = -0.7 # lower angular velocity constraint
	ubO   = 0.7 # upper angular velocity constraint

	params = {
		'alpha_w': alpha,
		'beta' : beta,
		'lbO'  : lbO,
		'ubO'  : ubO,
	}

	return params

def position_params():

	alpha1 = 2.5e1
	lbv    = -0.2 # lower velocity constraint
	ubv    = 0.2 # upper velocity constraint

	params = {
		'alpha_v': alpha1,
		'lbv'   : lbv,
		'ubv'   : ubv,
	}

	return params

def vision_parameters():

	b11 = 2.5e2
	b12 = 2.5e2
	b13 = 2.5e2
	b31 = 2.5e2
	b32 = 2.5e2
	b33 = 2.5e2
	lambda_gain = 0.5

	params = {
		'b11': b11,
		'b12': b12,
		'b13': b13,
		'b31': b31,
		'b32': b32,
		'b33': b33,
		'lambda_gain':lambda_gain,
	}

	return params


def orientation_params_simulations():

	alpha = 1e-2 # qp gain
	beta  = 1e2 # 2e-2 non linear 
	lbO   = -0.89 # lower angular velocity constraint
	ubO   = 0.89 # upper angular velocity constraint

	params = {
		'alpha_w': alpha,
		'beta' : beta,
		'lbO'  : lbO,
		'ubO'  : ubO,
	}

	return params

def position_params_simulations():

	alpha1 = 1e-2
	lbv    = -0.25 # lower velocity constraint
	ubv    = 0.25 # upper velocity constraint

	params = {
		'alpha_v': alpha1,
		'lbv'   : lbv,
		'ubv'   : ubv,
	}

	return params

def vision_parameters_simulations():

	b11 = 1e0
	b12 = 1e0
	b13 = 1e0
	b31 = 1e0
	b32 = 1e0
	b33 = 1e0

	params = {
		'b11': b11,
		'b12': b12,
		'b13': b13,
		'b31': b31,
		'b32': b32,
		'b33': b33,
	}

	return params

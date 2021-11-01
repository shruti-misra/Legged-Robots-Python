#Name: Shruti Misra	
#iLOR

import numpy as np
import numpy.linalg
import matplotlib.pyplot as plot 
from Hopper import Hopper
from HybridDynamics import HybridDynamics, obs, cost_final
import time

class iLQR:

	def __init__(self, model, cost):

		self.model = model
		self.cost = cost

		self.T = 10
		self.dt = 1e-3
		self.maxIter = 10

		self.lambda = Lambda
		self.lambda_factor = TODO
		self.dlambda = dLambda
		self.costThres = costThres

		self.hop = Hopper()
		self.p = self.hop.SysParamDef()
		self.y0 = self.hop.ContStateDef()
		self.z0 = self.hop.DiscStateDef()
		self.dt  = 1e-2
		self.rx = 1e-6
		self.K = 2
		self.T = 4
		self.U = np.zeros(int(T*(1/dt)))  

	def solve(self):

		'''1)Forward rollout:
			a)Compute overall cost of trj
			b) Compute derivatives

		   2)Backward pass
		   3)Evaluate new trj 

			A) For every time step simulate and slightly vary one of the params
		   '''
		pass


	def forward_pass(self, k, K, X, U):

		''' a) Compute overall cost of trj
			b) Compute derivatives '''

		#Simulate the trj - Get fx, fu
		#Compute cost  - Get lx, lu
		#Compute derivatives - lxx, lux, luu
		self.Lambda = Lambda
		self.lambda_factor = lambda_factor

		x_new = self.y0
		j_new = self.j0

		X_new = X_new.append(x_new)
		u_new = []
		cost = TODO #Compute OG cost

		for t in range(self.T):

			u = U[t] + k[t] + K[t]*(x_new(t) - X[t])
			x_new = TODO #from dynamics
			cost_new = TODO #Compute cost
			X_new.append(x_new)
			u_new.append(u)

		cost_new = TODO

		#Implement the Levenberg-Marquardt heuristic

		if cost_new < cost:

			#Decrease lambda and get closer to Newton's method

			Lambda /= lambda_factor
			XX = np.copy(X_new)
			UU = np.copy(U_new)
			oldcost = np.copy(cost)
			cost = np.copy(new_cost)
			#Simulate

		else:

			Lambda *= lambda_factor
			if Lambda > lambda_factor:

				print("Stuff")
				break


		return XX, UU, cost

	def backward_pass(self, X, U):

		'''
		Qx, Qu, Qxx, Qux, Quu
		k = -Quu(inv)Qu, K = -Quu(inv)Qux
		Vx = Qx - K(trans)Quuk
		Vxx = Qxx - K(trans)QuuK
		Return: Gains, k and K
		'''
		k = []
		K = []

		for i in range(self.T-1, -1):

			fx, fu, fxx, fux, fuu = TODO
			lx, lu, lxx, lux, luu = TODO
			Vx_next = TODO
			Vxx_next = TODO


			Qx = lx + np.dot(fx.T, Vx_next)
			Qu = lu + np.dot(fu.T, Vx_next)
			Qxx = lxx + np.dot(fx.T, np.dot(Vxx_next, fx)) + np.dot(Vx_next, fxx)
			Qux = lux + np.dot(fu.T, np.dot(Vxx_next, fx)) + np.dot(Vx_next, fux)
			Quu = luu + np.dot(fu.T, np.dot(Vxx_next, fu)) + np.dot(Vx_next, fuu)

			kt = np.dot(-np.linalg.inv(Quu), Qu)
			Kt = np.dot(-np.linalg.inv(Quu), Qux)

			Vx_next = Qx - np.dot(Kt.T,np.dot(Quu, kt))
			Vxx_next = Qxx - np.dot(Kt.T,np.dot(Quu, Kt))

			#Maybe add regularization

			K .append(Kt)
			k.append(kt)

		K.reverse()
		k.reverse()

		return K, k

	def compute_cost(self, y, z):

		#Define parameters that will go in the hybrid Dynamics sim
		p =  self.p
		dt = self.dt
		rx = self.rx
		K =  self.K 
		T =  self.T 
		U =  self.U

		#Compute trj and cost
		trj, eventVal, cost = HybridDynamics(y, z, p, K, T, rx,dt, U, calc_cost)

		#Unpack trj object
		k,t,j,x_d = obs(trjs,True, False, None, p)

		#Get the states explicitly
		for x in x_d:
    		x1.append(x[0])
    		x2.append(x[1])
    		x3.append(x[2])
    		x4.append(x[3])

    	final_cost, lx_final, lu_final, lxx_final, lu_final = cost_final(x2[len(x2) - 1])
    	total_cost = cost + final_cost
    	
    	return x_d, total_cost

	def compute_derivatives(self):

		pass







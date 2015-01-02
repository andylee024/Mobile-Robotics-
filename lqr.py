import matplotlib.pyplot as plt
import math
import numpy as np

class LQR_Matrices():
	def __init__(self):
		
		#Given Matrices 
		self.A = np.matrix([
			[1.0,0.01],
			[0.0,1.0]])
		
		self.B = np.matrix([
			[0.0],
			[0.02]
			])
		
		self.Q = np.matrix([
			[1.0, 0.0],
			[0.0, 0.1]
			])

		self.R = 0.01


def compute_K(A,B,V,R): return (B.T*V*B + R).I * (B.T*V*A)
def compute_V(A,B,R,Q,K,V): return Q + K.T*R*K + (A - B*K).T * V * (A - B*K)


"""
Computes gain matrices given set of initial matrices M 
Returns a list of proportional and derivative gains [(k_{p,t}, k_{d,t})] as well as list of matrices 
"""

def gain_matrices(M,n):

	"""initialize"""
	A,B,Q,R = M.A, M.B, M.Q, M.R # rename matrices
	K,V = np.matrix([[0.0,0.0]]), Q #initialize n, K_n, V_n

	#list of matrices 
	K_l = [K]
	V_l = [V]
	pd_gains = []
	pd_gains.append( (0,0) )

	""" Calculate Gain Matrices """ 
	while n >= 2:
		#compute K_{n-1} and V_{n-1}
		K = compute_K(A,B,V,R)
		V = compute_V(A,B,R,Q,K,V)
		
		# update lists and counter
 		K_l.append(K)
 		V_l.append(V)
 		pd_gains.append( (K[0,0],K[0,1]) ) #part a
		n = n-1

	""" Part a - Reorder list """
	#reverse lists 
	pd_gains.reverse()
	K_l.reverse()
	V_l.reverse()

	p_gains,d_gains = [0],[0]

	for i in range(len(pd_gains)):
		p_gains.append(pd_gains[i][0])
		d_gains.append(pd_gains[i][1])
 
	""" Plot Graphs - part a """ 
	#plt.plot(range(101),p_gains,'bo-', range(101),d_gains,'gs-')
	#plt.show() 

	return K_l,V_l


def controller(M, K_l,V_l):
	"""Initialize"""
	A,B,Q,R = M.A, M.B,M.Q,M.R
	q = np.matrix([[1.0, 10.0]]).T

	#list of states for each time step
	state = [(0,0)]
	state.append( (q[0,0],q[1,0]) )

	J = 0
	J_1 = q.T * V_l.pop(0) * q
	
	"""populate states"""
	while len(K_l) != 0:
		K = K_l.pop(0)
		u = -K * q

		#update J with q_t and u_t
		J = J + q.T*Q*q + u.T*R*u

		#obtain #q_{t+1}
		q = A*q + B*u
		state.append( (q[0,0], q[1,0]) )

	#compare cost functions Js
	print J, J_1

	"""part b - graphs"""
	#q_pos,q_vel = [],[]
	#for i in range(len(state)):
	#	q_pos.append(state[i][0])
	#	q_vel.append(state[i][1])

	
	#plot
	#plt.plot(range(101),q_pos,'bo-') #position
	#plt.plot(range(101),q_vel,'gs-') #velocity
	#plt.show() 


def main():
	#initialize matrices
	"""
	M = LQR_Matrices();
	K_l,V_l = gain_matrices(100,M)
	controller(M, K_l, V_l)
	"""

	#part c 
	# We note that they are all nearly the same 
	M = LQR_Matrices();
	K_l,V_l = gain_matrices(M,100)
	print "K_1 for n = 100" 
	print K_l[0]
	
	K_l,V_l = gain_matrices(M,1000)
	print "K_1 for n = 1000"  
	print K_l[0]
	
	K_l,V_l = gain_matrices(M,10000)
	print "K_1 for n = 10000" 
	print K_l[0]

	
	



	



if __name__ == '__main__':
  main()

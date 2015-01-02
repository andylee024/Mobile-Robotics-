"""
E28 HW6 - Simulated Pure Pursuit 
Author: Andy Lee
Date: 10/21/14

Comments: 

"""
import matplotlib.pyplot as plt
import math
import numpy as np

#transformation from robot to world 
def T_RW(R):
	
	#plug into transformation matrix 
	A = np.matrix([
		[np.cos(R.theta), -np.sin(R.theta), R.x],
		[np.sin(R.theta), np.cos(R.theta), R.y],
		[0.0,0.0,1.0]
		])
	return A

class Robot():
	def __init__(self,pos,param):
		self.x = pos[0]
		self.y = pos[1]
		self.theta = pos[2]

		self.a = param[0]
		self.k_x = param[1] 
		self.k_t = param[2]


def motion(R, t_step, clamp = False): 
	
	#initiate variables
	t = 0 
	list_x = [] #tracks x_pos for each t_step
	list_y = [] #tracks y_pos for each t_step
	
	while  t<=30:
		#map robot_origin to world frame - works!
		R_o = np.matrix([[0.0,0.0,1.0]]).T
		R_w = T_RW(R)*R_o

		#Calculate pure-pursuit values 
		p_c = np.matrix([[R_w[0,0],0.0,1.0]]).T
		p_d = np.matrix([[p_c[0,0]+R.a,0.0,1.0]]).T
		
		#map pursuit point p_d back to robot frame 
		pt_pursue = T_RW(R).I * p_d
		(u,v) = (pt_pursue[0,0],pt_pursue[1,0])

		#Apply Control Scheme
		angular_vel = R.k_t*(v/u)
		
		#clamp condition for simulation 3
		if clamp == True:
			if angular_vel > 0.15:
				angular_vel = 0.15
			elif angular_vel < - 0.15:
				angular_vel = 0.15

		dtheta = angular_vel * t_step
		R.theta += dtheta

		#update change in position in world frame 
		R_w = T_RW(R)*R_o
		R.x += R.k_x*(math.cos(R.theta)) * t_step  
		R.y += R.k_x*(math.sin(R.theta)) * t_step
		list_x.append(R.x)
		list_y.append(R.y)
		
		#update time
		t += t_step
	
	return (list_x,list_y)
	

	
def main():
	
	#simulation 1 - OK!
	r_pos = [0.0,-0.5,0.0] #[x, y, theta]
	r_param = [0.2,0.1,2.0] #[alpha, k_x, k_theta]
	R = Robot(r_pos,r_param)
	x_1,y_1 = motion(R, 0.01)

	#simulation 2a - alpha = 0.05 -OK! 
	r_param[0] = 0.05
	R= Robot(r_pos,r_param)
	x_2a,y_2a = motion(R, 0.01)
	
	#simulation 2b - alpha = 1.0 -OK! 
	r_param[0] = 1.0
	R= Robot(r_pos,r_param)
	x_2b,y_2b = motion(R, 0.01)

	
	#simulation 3 - bad - OK 
	r_param[0] = 0.2
	R= Robot(r_pos,r_param)
	x_3a,y_3a = motion(R, 0.01, True)

	#simulation 3 - fixed - OK 
	r_param[0] = 0.5
	R= Robot(r_pos,r_param)
	x_3b,y_3b = motion(R, 0.01, True)
	
	#plot 
	line1, = plt.plot(x_1, y_1, '-', color ='r', label = 't = 0.01')
	plt.legend()
	plt.axis([-3.0, 3.0, -3.0, 3.0])
	plt.xlabel('x-axis')
	plt.ylabel('y-axis')
	plt.show()

if __name__ == '__main__':
  main()

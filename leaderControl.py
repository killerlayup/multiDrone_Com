#!/usr/bin/python
"""
Adaptive control algorithm class for leaders, to keep two leaders matianing a constant distance 
@author: Peng Cheng
@date: 2015/9/3 18:52 
		
"""
import time

current_milli_time = lambda: int(time.time() * 1000)

class leaderControl(object):

	def __init__(self, vx0, vy0, initial_x, initial_y):
		self.vx0 = vx0
		self.vy0 = vy0
		self.vx = 0.0
		self.vy = 0.0
		self.x = initial_x
		self.y = initial_y

	# @param: x1-the x position of the object itself
	#         x2-the x position of another leader
	#         y1-the y position of the object itself
	#         y2-the y position of another leader
	def controller(self, x1, x2, y1, y2, d12):
		currentDisPow = pow(x1-x2, 2) + pow(y1-y2, 2)
		self.vx = self.vx0 + (x2 - x1) * (currentDisPow - pow(d12, 2)) * 0.02  #0.02 is arbitrary scale factor
		self.vy = self.vy0 + (y2 - y1) * (currentDisPow - pow(d12, 2)) * 0.02

	# It's better to output control in main function
	'''
	def run(self, vx, vy):
		send_ned_velocity(vx, vy, 0)  # vz = 0.0
	'''

if __name__ == "__main__":
	leader1 = leaderControl(1.0, 1.0, 0.0, 0.0)
	leader2 = leaderControl(1.0, 1.0, 10.0, 0.0)
	delt_T = 0.5  #s
	lastRecord = current_milli_time()
	cnt = 0
	while True:
		if current_milli_time() - lastRecord >= 500:  #ms
			print "[%d] current time is: " % cnt + str(current_milli_time())
			lastRecord = current_milli_time()
			leader1.controller(leader1.x, leader2.x, leader1.y, leader2.y, 2.0)
			leader2.controller(leader2.x, leader1.x, leader2.y, leader1.y, 2.0)

			leader1.x += leader1.vx * delt_T
			leader1.y += leader1.vy * delt_T
			leader2.x += leader2.vx * delt_T
			leader2.y += leader2.vy * delt_T

			cnt += 1

			print "x1:" + str(leader1.x)
			print "y1:" + str(leader1.y)
			print "x2:" + str(leader2.x)
			print "y2:" + str(leader2.y)
		else:
			pass
		
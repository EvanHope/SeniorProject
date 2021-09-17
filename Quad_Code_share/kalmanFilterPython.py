class kalmanFilterShawn:	
	
	def __init__(self):
		self.Q_angle = .001
		self.Q_bias = .003
		self.R_measure = .05
		
		self.angle = 0
		self.bias = 0
		
		self.P00 = 0
		self.P01 = 0
		self.P10 = 0
		self.P11 = 0

	def kalmanFilt(self,newAngle,newRate,dt):
		# step 1
		# update the a priori state estimate using
		# the a posteriori unbiased rate and bias
		# from the previous step
		self.rate = newRate - self.bias
		self.angle = self.angle + dt * (self.rate)
		
		#print self.angle

		# step 2
		# update the estimation error covariance matrix
		self.P00 += dt * (dt*self.P11 - self.P01 - self.P10 + self.Q_angle)
		self.P01 -= dt * self.P11
		self.P10 -= dt * self.P11
		self.P11 += self.Q_bias * dt

		# step 3
		# compute the innovation
		y = newAngle - self.angle

		# step 4
		# update innovation covariance matrix
		S = self.P00 + self.R_measure

		# step 5
		# compute the Kalman gain
		K0 = self.P00/S
		K1 = self.P10/S

		# step 6
		# compute the optimal estimate of the current state
		self.angle += K0 * y
		self.bias  += K1 * y

		# step 7
		# update the estimation error covariance
		P00_old = self.P00
		P01_old = self.P01

		self.P00 -= K0 * P00_old
		self.P01 -= K0 * P01_old
		self.P10 -= K1 * P00_old
		self.P11 -= K1 * P01_old

		return self.angle

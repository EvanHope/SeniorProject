# Shawn Herrington
# HW 2
# Code for Problem 2 and PRoblem 3


import math
# this all goes outside the main loop, initializing vars
rollGyro = 0 # initilize the roll measurment from the rate gyro
currentGyro = 0 # initilize the current gyro meas. var
previousGyro = 0 # initialize the previous gyro meas. var
rollAccel = 0 #initialize the roll measurement from the accel
rollEnc = 0 # initilize the roll measurement from the encoder
encoder0 = 0 # offset val for encoder

		# this goes in the 100Hz loop
		
		# set the current gyro data as the currentGyro value
		currentGyro = rates[1]
		
		if encoder0 == 0:
			encoder0 = analog[4] # this removes the encoder offset
		
		
		# use trapezoid rule to integrate the rate gyro data
		# this is in radians
		rollGyro = rollGyro + (.01*((currentGyro+previousGyro)/2.0))
		# use atan2 to calculate roll angle from accelerometer
		# this is in radians
		rollAccel = -math.atan2(accels[0], accels[2])
		# use the encoder to determine the real roll state
		# this is in degrees
		rollEnc = -(analog[4]-encoder0)*(360/5)
		
		# the first time this runs, previous gyro is initialized to
		# a value of 0; therefore, the first step does not give
		# meaningful data
		previousGyro = currentGyro # update the previous data value
		
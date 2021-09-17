def numIntegration(typ,timeStep,inputBuffer):
	# initialize the current estimate so that the program will run even
	# if a wrong type is chosen
	currentIntEst = 0
	# use Simpson's 1/3 rule, this estimate uses 3 data points and
	# must be handled more carefully in the calling program
	if(typ == "Simp"):
		currentIntEst = (timeStep/3.0)*(inputBuffer.prev(0)+4.0*inputBuffer.prev(1)+inputBuffer.prev(2))
	# use trapezoid rule, this estimate will work fine as called from
	# the calling program
	elif(typ == "Trap"):
		currentIntEst = (timeStep/2.0)*(inputBuffer.prev(0)+inputBuffer.prev(1))
	# this is the error condition
	else:
		print("Error:  type not supported")
	return currentIntEst
	
			

import math

def filterEval(coeff,order,inputBuffer,outputBuffer):
	filterOutput = 0
	for i in range(0,order+1):
		#print('multiply',coeff[i],'by',inputBuffer.prev(i),'index is',i)
		filterOutput = filterOutput + coeff[i]*inputBuffer.prev(i)
	for i in range(0,order):
		#print('multiply',coeff[order+1+i],'by',outputBuffer.prev(i),'index is',i)
		filterOutput = filterOutput + coeff[order+1+i]*outputBuffer.prev(i)
	return filterOutput

def butterworth(order,typ,fc,fs):
	
	# if Lowpass is desired
	if(typ == "LPF"):
		# Butterworth LP 1st Order
		if(order == 1):
			print('using 1st order lp')
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma+1.0
			b0 = gamma
			b1 = b0
			b0 = b0/D
			b1 = b1/D
			a1 = (gamma-1)/D
			coeff = [b0,b1,-a1]
		# Butterworth LP 2nd Order
		if(order == 2):
			print('using 2nd order lp')
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma**2+2.0**(1.0/2.0)*gamma+1
			b0 = gamma**2.0
			b1 = 2.0*b0
			b2 = b0
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			a1 = (2.0*(gamma**2-1))/D
			a2 = (gamma**2-2.0**(1.0/2.0)*gamma+1)/D
			coeff = [b0,b1,b2,-a1,-a2]
		# Butterworth LP 3rd Order
		if(order == 3):
			print('using 3rd order lp')
			gamma = math.tan(((math.pi)*fc)/fs)
			D = gamma**3+2*gamma**2+2.0*gamma+1
			b0 = gamma**3.0
			b1 = 3.0*b0
			b2 = 3.0*b0
			b3 = b0
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			b3 = b3/D
			a1 = (3.0*gamma**3+2*gamma**2-2*gamma-3)/D
			a2 = (3.0*gamma**3-2*gamma**2-2*gamma+3)/D
			a3 = (gamma**3-2*gamma**2+2*gamma-1)/D
			coeff = [b0,b1,b2,b3,-a1,-a2,-a3]
		# Butterworth LP 4th Order
		if(order == 4):
			print('using 4th order lp')
			gamma = math.tan(((math.pi)*fc)/fs)
			alpha = -2.0*(math.cos((5.0*math.pi)/8.0)+math.cos((7.0*math.pi)/8.0))
			beta = 2.0*(1.0+2.0*math.cos((5.0*math.pi)/8.0)*math.cos((7.0*math.pi)/8.0))
			D = gamma**4+alpha*gamma**3+beta*gamma**2+alpha*gamma+1.0
			b0 = gamma**4
			b1 = 4.0*b0
			b2 = 6.0*b0
			b3 = 4.0*b0
			b4 = b0
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			b3 = b3/D
			b4 = b4/D
			a1 = (2.0*(2.0*gamma**4+alpha*gamma**3-alpha*gamma-2))/D
			a2 = (2.0*(3.0*gamma**4.0-beta*gamma**2.0+3.0))/D
			a3 = (2.0*(2.0*gamma**4-alpha*gamma**3+alpha*gamma-2))/D
			a4 = (gamma**4-alpha*gamma**3+beta*gamma**2-alpha*gamma+1)/D
			coeff = [b0,b1,b2,b3,b4,-a1,-a2,-a3,-a4]
	# if highpass is desired
	elif(typ == "HPF"):
		# Butterworth HP 1st Order
		if(order == 1):
			print('using 1st order hp')
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma+1.0
			b0 = 1
			b1 = -1
			b0 = b0/D
			b1 = b1/D
			a1 = (gamma-1)/D
			coeff = [b0,b1,-a1]
		# Butterworth HP 2nd Order
		if(order == 2):
			print('using 2nd order hp')
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma**2+2.0**(1.0/2.0)*gamma+1
			b0 = 1
			b1 = -2
			b2 = 1
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			a1 = (2.0*(gamma**2-1))/D
			a2 = (gamma**2-2.0**(1.0/2.0)*gamma+1)/D
			coeff = [b0,b1,b2,-a1,-a2]
		# Butterworth HP 3rd Order
		if(order == 3):
			print('using 3rd order hp')
			gamma = math.tan(((math.pi)*fc)/fs)
			D = gamma**3+2*gamma**2+2*gamma+1
			b0 = 1
			b1 = -3
			b2 = 3
			b3 = -1
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			b3 = b3/D
			a1 = (3.0*gamma**3+2.0*gamma**2-2.0*gamma-3)/D
			a2 = (3.0*gamma**3-2.0*gamma**2-2.0*gamma+3)/D
			a3 = (gamma**3-2.0*gamma**2+2.0*gamma-1)/D
			coeff = [b0,b1,b2,b3,-a1,-a2,-a3]
		# Butterworth HP 4th Order
		if(order == 4):
			print('using 4th order hp')
			gamma = math.tan(((math.pi)*fc)/fs)
			alpha = -2.0*(math.cos((5.0*math.pi)/8.0)+math.cos((7.0*math.pi)/8.0))
			beta = 2.0*(1.0+2.0*math.cos((5.0*math.pi)/8.0)*math.cos((7.0*math.pi)/8.0))
			D = gamma**4+alpha*gamma**3+beta*gamma**2+alpha*gamma+1.0
			b0 = 1.0/D
			b1 = -4.0/D
			b2 = 6.0/D
			b3 = -4.0/D
			b4 = 1.0/D
			a1 = (2.0*(2.0*gamma**4.0+alpha*gamma**3.0-alpha*gamma-2.0))/D
			a2 = (2.0*(3.0*gamma**4.0-beta*gamma**2.0+3.0))/D
			a3 = (2.0*(2.0*gamma**4.0-alpha*gamma**3.0+alpha*gamma-2.0))/D
			a4 = (gamma**4.0-alpha*gamma**3.0+beta*gamma**2.0-alpha*gamma+1.0)/D
			coeff = [b0,b1,b2,b3,b4,-a1,-a2,-a3,-a4]
	# error condition
	else:
		print("Error:  filter type not supported\n")
	
	# return the calculated coefficients
	return coeff

	
	
	
	
	

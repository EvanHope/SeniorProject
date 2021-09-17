import math

def filterEval(coeff,inputBuffer,outputBuffer):
	filterOutput = 0
	for i in range(0,size(inputBuffer)-1):
		filterOutput = filterOutput + coeff[i]*inputBuffer.prev(i)
	for i in range(0,size(outputBuffer)-1):
		filteroutput = filterOutput + coeff[size(inputBuffer)+i]*outputBuffer.prev(i)

def butterworth1LP(n,fc,fs,typ):
	
	# if Lowpass is desired
	if(typ == "LPF"):
		# Butterworth LP 1st Order
		if(n == 1):
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma+1.0
			b0 = gamma
			b1 = b0
			b0 = b0/D
			b1 = b1/D
			a1 = (gamma-1)/D
			coeff = [b0,b1,-a1]
		# Butterworth LP 2nd Order
		if(n == 2):
			gamma = math.tan((math.pi*fc)/fs)
			D = gamma**2+2.0**(1.0/2.0)*gamma+1
			b0 = gamma**2
			b1 = 2*b0
			b2 = b0
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			a1 = (2.0*(gamma**2-1))/D
			a2 = (gamma**2-2.0**(1.0/2.0)*gamma+1)/D
			coeff = [b0,b1,b2,-a1,-a2]
		# Butterworth LP 3rd Order
		if(n == 3):
			gamma = math.tan(((math.pi)*fc)/fs)
			D = gamma**3+2*gamma**2+2*gamma+1
			b0 = gamma**3
			b1 = 3*b0
			b2 = 3*b0
			b3 = b0
			b0 = b0/D
			b1 = b1/D
			b2 = b2/D
			b3 = b3/D
			a1 = 3*gamma**3+2*gamma**2-2*gamma-3
			a2 = 3*gamma**3-2*gamma**2-2*gamma+3
			a3 = gamma**3-2*gamma**2+2*gamma-1
		if(n == 4):
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
			a2 = (2.0*(3*gamma**4+beta*gamma**2+3))/D
			a3 = (2.0*(2*gamma**4-alpha*gamma**3+alpha*gamma-2))/D
			a4 = (gamma**4-alpha*gamma**3+beta*gamma**2-alpha*gamma+1)/D
			coeff = [b0,b1,b2,b3,b4,-a1,-a2,-a3,-a4]
	
	return coeff
	
def butterworth1HP(fc,fs,x,y):
	gamma = math.tan((math.pi*fc)/fs)
	D = gamma+1.0
	
	b0 = 1
	b1 = -1
	
	b0 = b0/D
	b1 = b1/D
	
	a1 = (gamma-1)/D
	
	coeff = [b0,b1,-a1]
	
	return coeff	

# fixed code so that filter coefficient function can be run only once
# to prevent recalculing coefficients at each call
def butterworth2LP(fc,fs):
	gamma = math.tan((math.pi*fc)/fs)
	D = gamma**2+2.0**(1.0/2.0)*gamma+1
	b0 = gamma**2
	b1 = 2*b0
	b2 = b0
	
	b0 = b0/D
	b1 = b1/D
	b2 = b2/D
	
	a1 = (2.0*(gamma**2-1))/D
	a2 = (gamma**2-2.0**(1.0/2.0)*gamma+1)/D
	
	coeff = [b0,b1,b2,-a1,-a2]
	
	return coeff

# fixed code so that filter coefficient function can be run only once
# to prevent recalculing coefficients at each call
def butterworth2HP(fc,fs):
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
	
	return coeff

def butterworth4LP(fc,fs):
	
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
	
	#print(b0,b1,b2,b3,b4)
	
	a1 = (2.0*(2.0*gamma**4+alpha*gamma**3-alpha*gamma-2))/D
	a2 = (2.0*(3*gamma**4+beta*gamma**2+3))/D
	a3 = (2.0*(2*gamma**4-alpha*gamma**3+alpha*gamma-2))/D
	a4 = (gamma**4-alpha*gamma**3+beta*gamma**2-alpha*gamma+1)/D
	
	#print(a1,a2,a3,a4)
	
	coeff = [b0,b1,b2,b3,b4,-a1,-a2,-a3,-a4]
	
	return coeff

def butterworth4HP(fc,fs):
	
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
	a3 = (2.0*(2.0*gamma**4.0+alpha*gamma**3.0+alpha*gamma-2.0))
	a4 = (gamma**4.0-alpha*gamma**3.0+beta*gamma**2.0-alpha*gamma+1.0)/D
	
	coeff = [b0,b1,b2,b3,b4,-a1,-a2,-a3,-a4]
	
	return coeff	
	
	
	
	
	

### outside of the main loop		
import filterShawn

fc = .5
fs = 100.0
x = [0, 0, 0, 0]
y = [0, 0, 0]
x1 = [0, 0, 0, 0]
y1 = [0, 0, 0]

rollGyro = 0
rollAccel = 0
rollEnc = 0
encoder0 = 0

		
		### INSIDE OF STUDENT SECTION, 100Hz Loop
		
		x[3] = rates[1]
		
		if encoder0 == 0:
			encoder0 = analog[4]
		
		# determine roll based on accelerometer data
		rollAccel = -math.atan2(accels[0], accels[2])
		x1[3] = rollAccel
		rollEnc = -(analog[4]-encoder0)*(360/5)
		
		# filter gyro data
		y[2] = filterShawn.butterworth2HP(fc,fs,x,y)
		
		# apply trapezoid rule to filtered data
		rollGyro = rollGyro + (.01*((y[2]+y[1])/2.0))
		
		x[0] = x[1]
		x[1] = x[2]
		x[2] = x[3]
		
		y[0] = y[1]
		y[1] = y[2]
		
		
		# filtered acc data
		y1[2] = filterShawn.butterworth2LP(fc,fs,x1,y1)
		
		# update input and output lists
		x1[0] = x1[1]
		x1[1] = x1[2]
		x1[2] = x1[3]
		
		y1[0] = y1[1]
		y1[1] = y1[2]
		
		### END OF STUDENT SECTION
		
### In Separate Function File

### This is ported directly from my Matlab code.  In looking at
### the way it runs on the Navio/Pi hardware, it is computationally
### inefficient because it is calculating the coefficients at each
### call.  This will be fixed at a later date since the filter was
### working for now.

def butterworth2LP(fc,fs,x,y):
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
	
	currFiltEst = b0*x[2]+b1*x[1]+b2*x[0]-a1*y[1]-a2*y[0]
	
	return currFiltEst

def butterworth2HP(fc,fs,x,y):
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
	
	currFiltEst = b0*x[2]+b1*x[1]+b2*x[0]-a1*y[1]-a2*y[0]
	
	return currFiltEst
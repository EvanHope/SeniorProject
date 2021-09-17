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

# run these only once to calculate the coefficients
coeffGyro = filterShawn.butterworth2HP(fc,fs)
coeffAcc = filterShawn.butterworth2LP(fc,fs)

		
		### INSIDE OF STUDENT SECTION, 100Hz Loop
		
		# this is the gyro data
		# read it into the last position in the input que
		x[3] = rates[1]
		
		if encoder0 == 0:
			encoder0 = analog[4]
		
		# determine roll based on accelerometer data
		# this is roll angle using the accelerometer
		rollAccel = -math.atan2(accels[0], accels[2])
		
		# put the roll accelerometer angle on the input que
		x1[3] = rollAccel
		rollEnc = (analog[4]-encoder0)*(360/5)
		#rollEnc = analog[4]*(360.0/5.0)
		
		# filter gyro data
		y[2] = filterShawn.filter2Eval(coeffGyro,x,y)
		
		# apply trapezoid rule to filtered data
		rollGyro = rollGyro + (.01*((y[2]+y[1])/2.0))
		
		# update input and output que for gyro
		# gyro raw input
		x[0] = x[1]
		x[1] = x[2]
		x[2] = x[3]
		# gyro filtered output (not roll angle)
		y[0] = y[1]
		y[1] = y[2]
		
		# filter acc data using coefficients found earlier
		y1[2] = filterShawn.filter2Eval(coeffAcc,x1,y1)
		rollAccel = y1[2]
		
		# update input and output que for acc
		# accelerometer raw input
		x1[0] = x1[1]
		x1[1] = x1[2]
		x1[2] = x1[3]
		# accelerometer filtered output
		y1[0] = y1[1]
		y1[1] = y1[2]
		
		### END OF STUDENT SECTION
		
### In Separate Function File (filterShawn.py)

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
	
	coeff = [b0,b1,b2,a1,a2]
	
	return coeff

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
	
	coeff = [b0,b1,b2,a1,a2]
	
	return coeff

def filter2Eval(coeff,x,y):
	return coeff[0]*x[2]+coeff[1]*x[1]+coeff[2]*x[0]-coeff[3]*y[1]-coeff[4]*y[0]
import time
import spidev
import math
import argparse 
import sys
import navio.util
#import AccelGyroMag
import navio.mpu9250 # For magnetometer calibration only
import madgwickahrs.madgwickahrs as attitude
import navio.pwm
import navio.adc
import navio.leds
#import GPS
import numpy as np
import navio.ms5611
import os
import navio.rcinput
from transforms3d.euler import quat2euler
import filterShawn
import bufferShawn
import numIntegration
import kalmanFilterPython

# ----------------------------------------------------------------------
# Nic and Shawn's variables -------------------------------START--------
# ----------------------------------------------------------------------
convFact = 180/math.pi

def rad2Deg(rad):
	return rad*convFact
def deg2Rad(deg):
	return deg/convFact

def rangeCoeff(xLims, yLims):
	coeffRange = [0,0]
	# calc slope and y intercept
	coeffRange[0] = float(yLims[1]-yLims[0])/float(xLims[1]-xLims[0])
	coeffRange[1] = float(yLims[1]-xLims[1]*coeffRange[0])
	#print(coeffRange)
	return coeffRange

def rangeD(inputD,rangeCoeff):
	return inputD*rangeCoeff[0] + rangeCoeff[1]

rc0in = [1065,1962]
rc1in = [1065,1960]
rc2in = [1065,1970]

#rc0in = [1068, 1930] # roll, ailerons
#rc1in = [1067, 1930] # pitch, elevators
#rc2in = [1073, 1925] # throttle
#rc3in = [1072, 1922] # yaw, rudder
#rc4in = [1065, 1933] # 3 pos flight mode switch, C
#rc5in = [1065, 1933] # extra sliders, can be used for tuning
#rc6in = [1065, 1933] # extra sliders, can be used for tuning
#rc7in = [1065, 1933] # 3 position switch, E
#rc8in = [1065, 1933] # 2 position switch, A

#prollIn = [-1000,1000]

rc0out = [-30,30]
rc1out = [.250,.500]
#rc0outI = [.16,1.6]
rc2out = [.99,1.75]
#rc5out = [.001,.01]
#rc6out = [.01,1]

throttle = 1.1

prollOut = [-.15,.15]

rc0c = rangeCoeff(rc0in,rc0out)
rc1c = rangeCoeff(rc1in,rc1out)
#rc0cI = rangeCoeff(rc0in,rc0outI)
rc2c = rangeCoeff(rc2in,rc2out)
#rc5c = rangeCoeff(rc5in,rc5out)
#rc6c = rangeCoeff(rc6in,rc6out)

# set this flag to true to run the freq sweep, otherwise disable
excitation = False
zeroed = False

# set this flag to run the rc calibration (output all values to record max/min)
# rcCal = false

# yaw gains
#kpy = 19.39823
#kiy = .387965
#kdy = 7.525825
kpy = 0.4
kiy = 0.05
kdy = 0.0001

# ZN tuning
kp = .12057
ki = .321092
kd = .011319

# Disnmore Herrington Tuning
#kp = .508
#ki = .00001
#kd = .0897

# Disnmore Herrington Tuning using this now
#kp = 0.25
#ki = 0.0
#kd = 0.0


# Kevin Tuning
#kp = .28
#ki = 0
#kd = 3.0
kp = 0.295
ki = 0.0015
kd = 3.2

# Disnmore Herrington Tuning RollSimp
#kp = .05
#ki = .0
#kd = .000

# no overshoot ALTITUDE
kpz = .04019
kiz = .107031
kdz = .010061

# debug message flag
dbgmsg = True # turn on for specific messages
dbgmsg2 = False # both true to turn on all messages

# filter cutoff frequency and sampling rate
fc = 6
fs = 100.0
order = 2

# run these only once to calculate the coefficients
coeffGyro = filterShawn.butterworth(order,"HPF",fc,fs)
coeffAcc = filterShawn.butterworth(order,"LPF",fc,fs)

kalmanObj = kalmanFilterPython.kalmanFilterShawn()

# create input buffers for gyro raw and accel roll data
gyroRawBuffer = bufferShawn.bufferShawn(order+1)
accelRollBuffer = bufferShawn.bufferShawn(order+1)

# create output buffers for filtered gyro raw and filtered accel roll data
gyroFiltBuffer = bufferShawn.bufferShawn(order+2)
accelFiltBuffer = bufferShawn.bufferShawn(order)

# create roll error buffer
rollErrorCurr = 0
rollErrorPrev = 0
rollErrorSum = 0

pitchErrorPrev = 0
pitchErrorSum = 0

timeStep = .01 # seconds (at 100 Hz)

# initialize vars for Simpsons rule integration
evenRoll = 0
oddRoll = 0
rollGyroSimp = 0
sinr1 = 0
sinr2 = 0
sinr3 = 0
sinr4 = 0
rollKalman = 0
rollGyro = 0
rollGyroRaw = 0
currentGyro = 0
previousGyro = 0
rollAccel = 0
Pyaw = 0

yawStep = False
yawRel = 0
yawOffset = 0

stepInput = False
counter = 0

prev_alt = 0


# set up encoder variables, encoder0 is used to set the starting position to 0 degrees
rollEnc = 0
encoder0 = 0

if(excitation):
	n = 0
	A = .04
	wn = 1
	zeta = 1
	kp = wn**2
	kd = 2*zeta*wn
else:
	zeta = 1
	wn = 1
	#B = 51.3341 # experimentally determined, in radians
	B = 2941.2 # experimentally determined, in degrees
	A = -1.5856 # experimental
	
	wn = 4.96 # experimental 

	#kp = wn**2.0
	#kd = 2.0*zeta*wn

# ----------------------------------------------------------------------
# Nic and Shawn's variables --------------------------------END---------
# ----------------------------------------------------------------------


# current_alt has been zeroed out to prevent the code from wigging out
current_alt = 0



# Getting next available log file number and creating header#
gg = 0
#while os.path.exists("Log_Files/datalog%s.csv" % gg):
#	gg+=1
#header_string = "rates, motor right, motoro left\n"
#header_string = "Time, roll, rollr,motor right,motor left,msn1,msn2\n"
#fh = open("Log_Files/datalog%s.csv" % gg,"a")
#fh.write(header_string)
#fh.close()


print ("Initializing ADC")
navio.util.check_apm()
adc = navio.adc.ADC()
analog = [0] * adc.channel_count

print ("Initializing Sensors")
imu = navio.mpu9250.MPU9250()
imu.initialize()
rcin = navio.rcinput.RCInput()
AHRS_data = attitude.AHRS(0.01) #100Hz sensor attitude estimation FIXED 
## GPS Disabled
#GPS = GPS.U_blox()
#for ind in range(0, 10):
#	GPS.enable_posllh()
led = navio.leds.Led()
baro = navio.ms5611.MS5611()
baro.initialize()
time.sleep(0.25)
baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()
baro.refreshTemperature()
time.sleep(0.01) # Waiting for temperature data ready 10ms
baro.readTemperature()
baro.calculatePressureAndTemperature()
ground_alt = 0
target_alt = 10 #target alt experimentation.
#if baro.PRES < 1013: # small check in case barometer pressure is invalid
	#ground_alt = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)
	#print("altitude ground?:" + ground_alt)
	#target_alt = target_alt + ground_alt
	#print("altitude target?:" + target_alt)
	#ground_alt = 0
led.setColor('Red')
time.sleep(1)


if imu.testConnection():
    print ("Connection established: True")
else:
    sys.exit("Connection established: False")
    
accels, rates, m9m = imu.getMotion9()
if m9m[0] == 0:
	print ("WARNING: Mag reading zeros, try rebooting Navio")
	led.setColor('Magenta')

mag_calibration_flag = False # True means run calibration, 
							 # otherwise use data in 
							 # mag_calibration_constants.txt

 

# Run Magnetometer calibration if flag is TRUE
if mag_calibration_flag == True:
	led.setColor('Yellow')
	imu.mag_calibrate()

# Configure servo output
motor_front_pin = 0
motor_back_pin = 1
motor_left_pin = 2
motor_right_pin = 3
motor_front_pwm = navio.pwm.PWM(motor_front_pin)
motor_back_pwm = navio.pwm.PWM(motor_back_pin)
motor_left_pwm = navio.pwm.PWM(motor_left_pin)
motor_right_pwm = navio.pwm.PWM(motor_right_pin)
motor_front_pwm.initialize()
motor_back_pwm.initialize()
motor_left_pwm.initialize()
motor_right_pwm.initialize()
motor_front_pwm.set_period(200)
motor_back_pwm.set_period(200)
motor_left_pwm.set_period(200)
motor_right_pwm.set_period(200)

# Cycling the pwm commands to initialize the devices
motor_front_pwm.set_duty_cycle(1.000)
motor_back_pwm.set_duty_cycle(1.000)
motor_left_pwm.set_duty_cycle(1.000)
motor_right_pwm.set_duty_cycle(1.000)
time.sleep(0.5)
motor_front_pwm.set_duty_cycle(2.000)
motor_back_pwm.set_duty_cycle(2.000)
motor_left_pwm.set_duty_cycle(2.000)
motor_right_pwm.set_duty_cycle(2.000)
time.sleep(0.5)
motor_front_pwm.set_duty_cycle(1.000)
motor_back_pwm.set_duty_cycle(1.000)
motor_left_pwm.set_duty_cycle(1.000)
motor_right_pwm.set_duty_cycle(1.000)
time.sleep(0.5)

# Timers to maintain constant cycle times 
timein = time.time()
prev_time = (time.time()-timein)*1000.0
timer_1hz = prev_time
timer_10hz = prev_time
timer_25hz = prev_time
timer_50hz = prev_time
timer_100hz = prev_time
baro_timer = 0
# Declaring variables for use in main loop
rc_data = rcin.read_all()
motor_front = 1.000
motor_back = 1.000
motor_left = 1.000
motor_right = 1.000
gyro_2 = 0
gyro_1 = 0
gyro_0 = 0
cur_time = 0
roll_angle_gyro = 0
gyro_2p = 0
gyro_1p = 0
gyro_0p = 0
pitch_angle_gyro = 0
rates2= [0, 0, 0]
accels2 = [0,0,0]
#------------------------------------------------#
###       Declare global variables here        ###

# ----------------------------perform RC calibration------------------------------
#doOver = True
#rcCal = int(input("RC Calibration, 1 or 0\n"))
#if(rcCal):
#	while(doOver):
#		doOver = False
#		print("Calibrate RC Inputs")
#		print("Minimum Values")
#		print("rc_data[0]",rc_data[0],"rc_data[1]",rc_data[1],"rc_data[2]",rc_data[2],"rc_data[3]",rc_data[3],"rc_data[4]",rc_data[4])
#		rc0[0] = rc_data[0]
#		rc1[0] = rc_data[1]
#		rc2[0] = rc_data[2]
#		rc3[0] = rc_data[3]
#		rc4[0] = rc_data[4]
#		print("put all sticks in max position")
#		dummy = int(input("press enter\n"))
#		print("Maximum Values")
#		print("rc_data[0]",rc_data[0],"rc_data[1]",rc_data[1],"rc_data[2]",rc_data[2],"rc_data[3]",rc_data[3],"rc_data[4]",rc_data[4])
#		rc0[1] = rc_data[0]
#		rc1[1] = rc_data[1]
#		rc2[1] = rc_data[2]
#		rc3[1] = rc_data[3]
#		rc4[1] = rc_data[4]
#		# these are the output limits for the RC inputs
#		rc0out = [-90,90]	
#		rc1out = [0,1]	
#		rc2out = [1,10]	
#		rc3out = [0,1]	
#		rc4out = [0,1]
#		# these are the coefficients for the RC input ranges
#		rc0c = rangeCoeff(rc0,rc0out)
#		rc1c = rangeCoeff(rc1,rc1out)
#		rc2c = rangeCoeff(rc2,rc2out)
#		rc3c = rangeCoeff(rc3,rc3out)
#		rc4c = rangeCoeff(rc4,rc4out)
#	doOver = int(input("THIS WILL ERASE ALL VALS\nto redo, type 1, otherwise type 0, then press enter\n"))


alts = 0
altitudeErrorSum = 0
altitudeErrorPrev = 0
yawErrorSum = 0
yawRateErrorPrev = 0
yawErrorPrev = 0

yaw,roll,pitch = quat2euler(AHRS_data.quaternion,axes='rzxy')
yawDes = yaw


print ("Starting main loop: here we go!")
while True:
	current_time = (time.time()-timein)*1000.0
	if m9m[0] == 0:
		led.setColor('Magenta')
	else:
		led.setColor('Green')
		
	if (current_time - timer_100hz) >=10.0: # 10 ms = 100Hz
		#print ("In 100Hz loop!")
		#### IMU/Attitude and GPS estimation: DO NOT TOUCH ####
		for i in range (0, adc.channel_count):
			analog[i] = adc.read(i)*0.001
		accels, rates, m9m = imu.getMotion9()
		accels2[0] = -accels[1]
		accels2[1] = -accels[0]
		accels2[2] = accels[2]
		rates2[0] = rates[1]
		rates2[1] = rates[0]
		rates2[2] = -rates[2]
		AHRS_data.update_imu(rates2, accels2)
		yaw,roll,pitch = quat2euler(AHRS_data.quaternion,axes='rzxy')
		yaw = rad2Deg(yaw) + 0.011
		yaw = deg2Rad(yaw)
		baro_timer = baro_timer + 1
		if (baro_timer == 1): baro.refreshPressure()
		elif (baro_timer == 2): baro.readPressure()
		elif (baro_timer == 3): baro.refreshTemperature()
		elif (baro_timer == 4): baro.readTemperature()
		elif (baro_timer == 5):
			baro.calculatePressureAndTemperature()
			baro_timer = 0
			#print baro.PRES
			#if baro.PRES < 1013: # Only update if barometer is valid
				#alts = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)
				#print "altitude?:"
				#print alts
				#alts = 0
				#prev_alt = current_alt
				#current_alt = alts - ground_alt
				#print current_alt
				
		#buffer = GPS.bus.xfer2([100])
		## GPS is disabled ##
		#for byt in buffer:
		#	GPS.scan_ubx(byt)
		#	if(GPS.mess_queue.empty() != True):
		#		GPS_data = GPS.parse_ubx()
		#### End DO NOT TOUCH ####
		
		#----------------------------------------------------#
		#### 			BEGIN STUDENT SECTION			 ####
		# This section runs at 100Hz. You can add things for executation
		# at other speeds. See the 1Hz loop for display examples. SD 
		# logging occurs at 10Hz. See the 10Hz if statement for details
		
		# Output commands
		# 		motor_front, motor_back, motor_left, motor_right are the 4 motor commands
		# 		motor range is from 1.000 to 2.000 (1.000 is 0% power)
		
		# R/C Input 
		# 		rc_data variable stores all RC inputs (range is [0]-[5])
		#		each rc_data channel varies between 1000 and 2000
		
		# Sensor data
		#		yaw, roll, and pitch contain attitude info (float)
		#		rates[0 to 2] stores the three angular velocity components (rad/s)
		#		accels[0 to 2] stores the three linear accelerations
		#		analog[4] and [5] are the two analog inputs (value is in Volts)
		#		current_alt contains the current altitude (relatively to 
		#		start up) in meters (from the barometer)
		
		# -------------------------ENCODER ROLL-----------------------------------------
		# if encoder0 == 0:
			# encoder0 = analog[4]		
		# rollEnc = (analog[4]-encoder0)*(360/5)
		# rollEnc = analog[4]*(360.0/5.0)
		# -------------------------ENCODER ROLL-----------------------------------------
		
		rc_data = rcin.read_all()
		
		# -------------------------ATTITUDE EST-----------------------------------------
		# put current gyro raw data on the head of the gyro input buffer
		#gyroRawBuffer.put(rates[1])
		#print(gyroRawBuffer.prev(0),rates[1])
		
		#rollKalman = kalmanObj.kalmanFilt(rad2Deg(-math.atan2(accels[0], accels[2])),rad2Deg(rates[1]),timeStep)
		
		# put the roll accelerometer angle on the head of the accel roll input buffer
		#accelRollBuffer.put(-math.atan2(accels[0], accels[2]))
		
		# filter gyro data		
		#gyroFiltBuffer.put(filterShawn.filterEval(coeffGyro,order,gyroRawBuffer,gyroFiltBuffer))
		
		# apply trapezoid rule to the filtered gyro data
		#rollGyro = rollGyro + numIntegration.numIntegration("Trap",timeStep,gyroFiltBuffer)
		
		# apply trapezoid rule to the filtered gyro data
		#rollGyroRaw = rollGyroRaw + numIntegration.numIntegration("Trap",timeStep,gyroRawBuffer)
		
		# apply simpson's rule to the filtered gyro data
		#if((-1.0)**i >=0):
		#	evenRoll = evenRoll + numIntegration.numIntegration("Simp",timeStep,gyroFiltBuffer)
		#	rollGyroSimp = evenRoll
		#else:
		#	oddRoll = oddRoll + numIntegration.numIntegration("Simp",timeStep,gyroFiltBuffer)
		#	rollGyroSimp = oddRoll
	
		# filter acc data using coefficients found earlier
		#accelFiltBuffer.put(filterShawn.filterEval(coeffAcc,order,accelRollBuffer,accelFiltBuffer))
		#rollAccel = accelFiltBuffer.prev(0)
		
		# -------------------------ATTITUDE EST-----------------------------------------
		
		
		
		
		# -------------------------STABILIZATION-----------------------------------------
		# read desired roll from RC stick

		#uncomment for pitch and roll controller control:
		#set rollDes and PitchDes to 0 to ensure drone always tries to stay stable
		rollDes = rangeD(float(rc_data[0]),rc0c)
		pitchDes = rangeD(float(rc_data[1]),rc0c)
		throttle = rangeD(float(rc_data[2]),rc2c)
		#rollDes = 0
		#pitchDes = 0
		#throttle = 1
		#throttle = 1.1 #for testing motors
		yawRateDes = 0
		#yawDes = 0 # this is set right before main loop temp
		
		if rollDes < 7 and rollDes >-7:
			rollDes = 0
		if pitchDes < 7 and pitchDes > -7:
			pitchDes = 0
		if throttle < 1.1:
			throttle = 1.0
		
		
		if(yawStep and counter > 300):
			Pyaw = 0.05								#### what is Pyaw?
		
		
		if(stepInput and counter>500):
			#print("this is happening")
			rollDes = 0
		
		if(stepInput and counter-1000 > 0):
			rollDes = 0
		
		if(stepInput and counter-1500 > 0):
			rollDes = 0
		
		if(stepInput and counter-2000 > 0):
			rollDes = 0
			counter = 0
		
		
		# uncomment for Kalman roll
		#rollError = rollDes - rollKalman
		#print rollKalman
		# uncomment for simpsons rule roll
		#rollError = rollDes - rad2Deg(rollAccel-rollGyro)
		
		# uncomment for onboard roll/pitch
		rollError = rollDes - rad2Deg(roll)
		pitchError = pitchDes - rad2Deg(pitch)
		yawRateError = yawRateDes - rates[2]
		yawError = rad2Deg(yawDes) - rad2Deg(yaw)
		#print(current_alt)
		#print(altitudeError)


		
		#print(rc_data)
		
		
		if(not excitation):
			# NDI control
			#Proll = (kd*rad2Deg(float(-rates[1])))/(B*1.8)+(kp*(rollError))/B-(A*rad2Deg(float(rates[1])))/(B*1.4)
			
			
			#yawProportional = kpy * deg2Rad(yawRateError)	
			yawProportional = kpy * deg2Rad(yawError)	

			#print(yawProportional)
			#derivative = kd * deg2Rad((rollError - rollErrorPrev)/timeStep)
			#yawDerivative = kdy * -deg2Rad(rates[2])
			yawDerivative = kdy * -deg2Rad(rates[2])
			#print(yawDerivative)
			#print(deg2Rad((rollError - rollErrorPrev)/timeStep))
			yawErrorSum = yawErrorSum + (yawError + yawErrorPrev)*(timeStep/2.0)
			#print(rollErrorSum)
			yawIntegral = kiy * deg2Rad(yawErrorSum)
			#print(yawIntegral)
			
			
			rollProportional = kp * deg2Rad(rollError)
			#derivative = kd * deg2Rad((rollError - rollErrorPrev)/timeStep)
			rollDerivative = kd * -deg2Rad(rates[1])
			#print(deg2Rad((rollError - rollErrorPrev)/timeStep))
			rollErrorSum = rollErrorSum + (rollError + rollErrorPrev)*(timeStep/2.0)
			#print(rollErrorSum)
			rollIntegral = ki * deg2Rad(rollErrorSum)
			
			pitchProportional = kp * deg2Rad(pitchError)
			#derivative = kd * deg2Rad((rollError - rollErrorPrev)/timeStep)
			pitchDerivative = kd * -deg2Rad(rates[0])
			#print(deg2Rad((rollError - rollErrorPrev)/timeStep))
			pitchErrorSum = pitchErrorSum + (pitchError + pitchErrorPrev)*(timeStep/2.0)
			#print(rollErrorSum)
			pitchIntegral = ki * deg2Rad(pitchErrorSum)
			
		# -------------------------Kill Switch------------------------------------
		# eveyrthing in here only happens when the switch is on (up)
		if(float(rc_data[4]) > 1700.0): #rc_data[4] is C and D on controller
		#if(1):
			timer = time.time() - timein
			if(rollErrorSum > .5):
				rollErrorSum = 0
			if(not zeroed):
				rollErrorSum = 0
				#yawOffset = yaw

			Proll = rollProportional+rollIntegral+rollDerivative
			Ppitch = pitchProportional+pitchIntegral+pitchDerivative
			Pyaw = yawProportional+yawIntegral+yawDerivative

			counter = counter + 1

			#motor_right = throttle - Proll
			#motor_left = throttle + Proll
			#motor_front = throttle + Ppitch
			#motor_back = throttle - Ppitch

			motor_right = throttle - Proll + Pyaw
			motor_left = throttle + Proll + Pyaw
			motor_front = throttle + Ppitch - Pyaw
			motor_back = throttle - Ppitch - Pyaw

			zeroed = True
		else:
			motor_right = 0
			motor_left = 0
			motor_front = 0
			motor_back = 0
			zeroed = False
			Pyaw  = 0
			counter = 0
		
		
		pitchErrorPrev = pitchError
		rollErrorPrev = rollError
		yawRateErrorPrev = yawRateError		
		yawPrev = yaw
		
		
		
		# LOG DATA
		
		# RC Controller INPUT #

				
		#print (x[4]*(180/math.pi),rollGyro*180/math.pi,x1[4]*(180/math.pi),rollAccel*180/math.pi)
				
		### Data logging feature ###
		# GPS is disabled, tab in fh and below to re-enable
		#try:
		#	GPS_data
		#except NameError:
		#	GPS_data = None
		#if GPS_data is not None:
		#fh = open("Log_Files/datalog%s.csv" % gg,"a")
		#log_data = np.array([time.clock(), GPS_data.lat/10000000.0, GPS_data.lon/10000000.0,
		#### 			LOGGING 				####
		# This is the data to be logged. The header (text at top of file) is edited at the top
		# of the program. Add/subtract variables as needed.
		#log_data = np.array([time.time()-timein,roll,rates[1],motor_right,motor_left,sinr1,sinr2])
		#np.savetxt(fh, log_data.reshape(1,log_data.shape[0]), delimiter=',', fmt='%.6f')
		
		#fh.close()
		
		
		
		####		 END STUDENT SECTION				####
		#---------------------------------------------------#


		motor_front_pwm.set_duty_cycle(motor_front)
		motor_back_pwm.set_duty_cycle(motor_back)
		motor_left_pwm.set_duty_cycle(motor_left)
		motor_right_pwm.set_duty_cycle(motor_right)
		timer_100hz = current_time # reset timer flag
		# end of 100Hz section
	
	if (current_time - timer_50hz) >= 20.0:
		
		
		
		
		timer_50hz = current_time
		
		# End of 50Hz section
	
	if (current_time - timer_25hz) >= 40.0:
		
		
		
		
		timer_25hz = current_time
		
		# End of 25Hz section
		
	if (current_time - timer_10hz) >= 100.0:
#		# RC Controller INPUT #
#		rc_data = rcin.read_all()
#		
		#if(dbgmsg and dbgmsg2):
		#	print("Gyro Filter Buffer Contents")
		#	print(gyroFiltBuffer)
		
		#print(rad2Deg(rollAccel+rollGyroSimp))
		#print(rad2Deg(accelRollBuffer.prev(0)),rad2Deg(accelFiltBuffer.prev(0)))
		#print(rad2Deg(rollGyroRaw),rad2Deg(gyroFiltBuffer.prev(0)))
		#print rc_data
		#print(wn)
		#print(rollError)
		#print(float(rc_data[2]))
		
		
		#print(rollAccel*(180/math.pi))
		
		#print((rollGyro+rollAccel)*(180.0/math.pi),(rollGyroSimp+rollAccel)*(180/math.pi))
		#print(rollEnc)
#		
#		#print (x[4]*(180/math.pi),rollGyro*180/math.pi,x1[4]*(180/math.pi),rollAccel*180/math.pi)
#		
#		### Data logging feature ###
#		# GPS is disabled, tab in fh and below to re-enable
#		#try:
#		#	GPS_data
#		#except NameError:
#		#	GPS_data = None
#		#if GPS_data is not None:
#		fh = open("Log_Files/datalog%s.csv" % gg,"a")
#		#log_data = np.array([time.clock(), GPS_data.lat/10000000.0, GPS_data.lon/10000000.0,
#		#### 			LOGGING 				####
#		# This is the data to be logged. The header (text at top of file) is edited at the top
#		# of the program. Add/subtract variables as needed.
#		log_data = np.array([time.clock(),current_alt, yaw, pitch, roll, 
#					accels[0], accels[1], accels[2], rates[0], rates[1], rates[2],
#					m9m[0], m9m[1], m9m[2], rates[0], rates[1], rates[2],accels[0],accels[1],accels[2],rollGyro, rollAccel, rollEnc, (rollGyro+rollAccel)*(180/math.pi)])
#		np.savetxt(fh, log_data.reshape(1,log_data.shape[0]), delimiter=',', fmt='%.6f')
#		
#		fh.close()
#		####			END LOGGING				####
#		
		timer_10hz = current_time
		# End of 10Hz section
	
	if (current_time - timer_1hz) >= 1000.0:
		# Customizable display message #
		#print "Angles:", "{:+3.2f}".format(roll*57.32), "{:+3.2f}".format(pitch*57.32), "{:+3.2f}".format(yaw*57.32)
		print("current roll:")
		print(rad2Deg(roll))
		print("roll error:")
		print(rollError)

		print("current pitch:")
		print(rad2Deg(pitch))
		print("pitch error:")
		print(pitchError)

		print("yaw: ",rad2Deg(yaw))
		print("Yaw Error: ", yawError)
		print("Yaw Desired:", rad2Deg(yawDes))
		print("Pyaw: ",Pyaw)
		print("YawPorportional: ", yawProportional)
		print("yaw rate:", rad2Deg(rates[2]))


		print("right motor value:")
		print (motor_right)
		print("left motor value:")
		print (motor_left)
		print("front motor value:")
		print (motor_front)
		print("back motor value:")
		print (motor_back)
		#print "Analogs:", analog[0], analog[1], analog[2], analog[3], analog[4]
		#print "Altitude:", current_alt
		#print pitch_angle_gyro
		#print roll_angle_acc
		#print roll_angle_gyro
		
		#if GPS_data is not None:
		#	print "Location:", "{:+3.6f}".format(GPS_data.lat/10000000.0), "{:+3.6f}".format(GPS_data.lon/10000000.0), "{:+4.1f}".format(GPS_data.heightSea/1000.0)
		#	print "Loc Accuracy:", "{:+3.3f}".format(GPS_data.horAcc/1000.0), "{:+3.3f}".format(GPS_data.verAcc/1000.0)
		#print pitch_angle_gyro
		#print accels
		timer_1hz = current_time
		# End of 1Hz section

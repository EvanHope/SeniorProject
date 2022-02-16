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
#import serial
import struct
#port = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout=3.0)
# Getting next available log file number and creating header#
gg = 0
while os.path.exists("Log_Files/datalog%s.csv" % gg):
	gg+=1
header_string = "Time, Altitude, Yaw, Pitch, Roll, Ax, Ay, Az, Pitch_rate, Roll_rate,Yaw_rate, Magx, Magy, Magz"
fh = open("Log_Files/datalog%s.csv" % gg,"a")
fh.write(header_string)
fh.close()


print ("Initializing ADC")
navio.util.check_apm()
adc = navio.adc.ADC()
analog = [0] * adc.channel_count

print ("Initializing Sensors")
imu = navio.mpu9250.MPU9250()
imu.initialize()
#time.sleep(10)
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
'''if baro.PRES < 1013: # small check in case barometer pressure is invalid
	ground_alt = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)'''

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
timeg = time.time() - timein
prev_time = timeg*1000.0
timer_1hz = prev_time
timer_10hz = prev_time
timer_25hz = prev_time
timer_50hz = prev_time
timer_100hz = prev_time
timer_1000hz = prev_time
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
ac_AHRS = [0,0,0]
gy_AHRS = [0,0,0]
mag_AHRS = [0,0,0]
xpos = 0;ypos = 0;zpos = 0
wn = 4; damp = 1;kd = 2*damp*wn; kp = wn*wn; # controller gains
c = -0.15;b = 0.15 # motor constants
yawd = 0; coryaw = 0;yawddprev = 0;angleyaw = 0;n = 0;yaw1cmd = 0;yaw2cmd = 0;yaw3cmd = 0;yaw4cmd = 0


#------------------------------------------------#
###       Declare global variables here        ###

print ("Starting main loop: here we go!")
while True:
	current_time = (time.time() - timein)*1000.0
	#print (time.time())
	if m9m[0] == 0:
		led.setColor('Magenta')
	else:
		led.setColor('Green')
		
		
	if (current_time - timer_1000hz) >=1.0: # 1 ms = 1000Hz
		#### IMU/Attitude and GPS estimation: DO NOT TOUCH ####
		for i in range (0, adc.channel_count):
			analog[i] = adc.read(i)*0.001
		accels, rates, m9m = imu.getMotion9()
		ac_AHRS[0] = -accels[1]
		ac_AHRS[1] = -accels[0]
		ac_AHRS[2] = accels[2]
		gy_AHRS[0] = rates[1]
		gy_AHRS[1] = rates[0]
		gy_AHRS[2] = -rates[2]
		AHRS_data.update_imu(gy_AHRS, ac_AHRS)
		#yaw,pitch,roll = quat2euler(AHRS_data.quaternion,axes='rzxy')
		yaw,roll,pitch = quat2euler(AHRS_data.quaternion,axes='rzxy')
		baro_timer = baro_timer + 1
		if (baro_timer == 1): baro.refreshPressure()
		elif (baro_timer == 2): baro.readPressure()
		elif (baro_timer == 3): baro.refreshTemperature()
		elif (baro_timer == 4): baro.readTemperature()
		elif (baro_timer == 5):
			baro.calculatePressureAndTemperature()
			baro_timer = 0
			#print baro.PRES
			if baro.PRES < 1013: # Only update if barometer is valid
				#alts = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)
				alts = 0
				current_alt = alts - ground_alt
		
		#buffer = GPS.bus.xfer2([100])
		## GPS is disabled ##
		#for byt in buffer:
		#	GPS.scan_ubx(byt)
		#	if(GPS.mess_queue.empty() != True):
		#		GPS_data = GPS.parse_ubx()
		#### End DO NOT TOUCH ####
		
		#----------------------------------------------------#
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
		#rollDes = rangeD(float(rc_data[0]),rc0c)
		#pitchDes = rangeD(float(rc_data[1]),rc0c)
		#set to 0 to ensure drone always tries to stay stable
		rollDes = rangeD(float(rc_data[0]),rc0c)
		pitchDes = rangeD(float(rc_data[1]),rc0c)
		throttle = rangeD(float(rc_data[2]),rc2c)
		#throttle = 1.1 #for testing motors
		yawRateDes = 0
		
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
		altitudeError = target_alt - current_alt
		#print(current_alt)
		#print(altitudeError)

		
		#print(rc_data)
		
		#wn = rangeD(float(rc_data[2]),rc2c)
		# recalculate at each time step to account for changing wn
		#kp = wn**2.0
		#kd = 2.0*zeta*wn
		
		#kd = rangeD(float(rc_data[2]),rc2c)
		#print kd
		
		#kd = rangeD(float(rc_data[5]),rc5c)
		#kp = rangeD(float(rc_data[6]),rc6c)
		#print kp, kd
		
		if(not excitation):
			# NDI control
			#Proll = (kd*rad2Deg(float(-rates[1])))/(B*1.8)+(kp*(rollError))/B-(A*rad2Deg(float(rates[1])))/(B*1.4)
			
			
			#yawProportional = kpy * rates[2]
			#derivative = kd * deg2Rad((rollError - rollErrorPrev)/timeStep)
			#yawDerivative = kdy * -rates[2]
			#print(deg2Rad((rollError - rollErrorPrev)/timeStep))
			#yawErrorSum = yawErrorSum + (yawError + yawErrorPrev)*(timeStep/2.0)
			#print(rollErrorSum)
			#yawIntegral = ki * deg2Rad(yawErrorSum)
			
			
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

			altitudePorportional = kpz * altitudeError

			alt_velocity = abs(prev_alt - current_alt) / timeStep #Calculates alt_velocity WARNING: inaccurate measurement
			altitudeDerivative = kdz * -alt_velocity #alt_velocity may be inaccurate or wrong testing is needed

			altitudeErrorSum = altitudeErrorSum + (altitudeError + altitudeErrorPrev)*(timeStep/2.0) 

			altitudeIntegral = kiz * altitudeErrorSum
			
		# -------------------------Kill Switch------------------------------------
		# eveyrthing in here only happens when the switch is on (up)
		if(float(rc_data[4]) > 1700.0): #rc_data[4] is C and D on controller
		#if(1):
			timer = time.time() - timein
			if(rollErrorSum > .5):
				rollErrorSum = 0
			if(not zeroed):
				rollErrorSum = 0
				yawOffset = yaw

			Proll = rollProportional+rollIntegral+rollDerivative
			Ppitch = pitchProportional+pitchIntegral+pitchDerivative
			#Paltitude = altitudePorportional+altitudeIntegral+altitudeDerivative
			#print("this is Paltitude:" + Paltitude)
			#print rad2Deg(yawRel)
			
			counter = counter + 1
			if(excitation):
				timer=(time.time()-timein)
				frequencySweep = math.sin(2*math.pi*timer*(.2+.001*n))
				n=n+1
				
				motor_right = throttle - A * frequencySweep
				motor_left = throttle + A * frequencySweep
	
			else:
				#motor_right = 1.4 + sinr1 
				#motor_left = 1.4 + sinr2 
				#motor_front = 0
				#motor_back = 0
				motor_right = throttle - Proll
				#print (motor_right)
				motor_left = throttle + Proll
				#print (motor_left)
				motor_front = throttle + Ppitch
				#print (motor_front)
				motor_back = throttle - Ppitch
				#print (motor_back)
			zeroed = True
		elif(float(rc_data[4]) < 1700.0 and float(rc_data[4]) > 1600): #not sure if this is the correct values
			#altitude control enabled!!! WARNING
			timer = time.time() - timein
			if(rollErrorSum > .5):
				rollErrorSum = 0
			if(not zeroed):
				rollErrorSum = 0
				yawOffset = yaw

			Proll = rollProportional+rollIntegral+rollDerivative
			Ppitch = pitchProportional+pitchIntegral+pitchDerivative
			Paltitude = altitudePorportional+altitudeIntegral+altitudeDerivative
			#print("this is Paltitude:" + Paltitude)
			#print rad2Deg(yawRel)
			
			counter = counter + 1
			if(excitation):
				timer=(time.time()-timein)
				frequencySweep = math.sin(2*math.pi*timer*(.2+.001*n))
				n=n+1
				
				motor_right = throttle - A * frequencySweep
				motor_left = throttle + A * frequencySweep
	
			else:
				#motor_right = 1.4 + sinr1 
				#motor_left = 1.4 + sinr2 
				#motor_front = 0
				#motor_back = 0
				motor_right = Paltitude - Proll
				#print (motor_right)
				motor_left = Paltitude + Proll
				#print (motor_left)
				motor_front = Paltitude + Ppitch
				#print (motor_front)
				motor_back = Paltitude - Ppitch
				#print (motor_back)
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
		#altitudeErrorPrev = altitudeError
		
		
		
		
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

		print("target altitude:")
		print(target_alt)
		print("current altitude:")
		print(current_alt)
		print("altitude error:")
		print(altitudeError)


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

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



rc0out = [-30,30]
rc1out = [.250,.500]
#rc0outI = [.16,1.6]
rc2out = [.99,1.75]
#rc5out = [.001,.01]
#rc6out = [.01,1]

throttle = 1.5



# Getting next available log file number and creating header#
gg = 0
while os.path.exists("Log_Files/datalog%s.csv" % gg):
	gg+=1
#header_string = "rates, motor right, motoro left\n"
header_string = "Time, roll, rollr,motor right,motor left,msn1,msn2\n"
fh = open("Log_Files/datalog%s.csv" % gg,"a")
fh.write(header_string)
fh.close()


print "Initializing ADC"
navio.util.check_apm()
adc = navio.adc.ADC()
analog = [0] * adc.channel_count

print "Initializing Sensors"
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
if baro.PRES < 1013: # small check in case barometer pressure is invalid
	#ground_alt = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)
	ground_alt = 0
led.setColor('Red')
time.sleep(1)


if imu.testConnection():
    print "Connection established: True"
else:
    sys.exit("Connection established: False")
    
accels, rates, m9m = imu.getMotion9()
if m9m[0] == 0:
	print "WARNING: Mag reading zeros, try rebooting Navio"
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



print "Starting main loop: here we go!"
while True:
	current_time = (time.time()-timein)*1000.0
	if m9m[0] == 0:
		led.setColor('Magenta')
	else:
		led.setColor('Green')
		
	if (current_time - timer_100hz) >=10.0: # 10 ms = 100Hz
		print "In 100Hz loop!"
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
		roll,pitch,yaw = quat2euler(AHRS_data.quaternion,axes='rxyz')
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
				
		
		
		rc_data = rcin.read_all()
		

		motor_right = 1.5
		motor_left = 1.5
		motor_front = 1.5
		motor_back = 1.5
		counter = 0
		
		
		
		
		####		 END STUDENT SECTION				####
		#---------------------------------------------------#
		
		
		motor_front_pwm.set_duty_cycle(motor_front)
		motor_back_pwm.set_duty_cycle(motor_back)
		motor_left_pwm.set_duty_cycle(motor_left)
		motor_right_pwm.set_duty_cycle(motor_right)
		timer_100hz = current_time # reset timer flag
		# end of 100Hz section
######################################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko and Shez Malik
######################################################
#
# This code is written by Joshua and modified by Shez Malik 
# to create a lib that returns (MPU6050 - accel/gyro, AK8963 - mag) 
# data to be sent as a UDP packet by leveraging the functionality
# of the MPU9250_i2c.py library
#
######################################################
import time
class iframe:
	def __init__(self, IMUdata):
		self.accel_x = IMUdata[0]
		self.accel_y = IMUdata[1]
		self.accel_z = IMUdata[2]
		self.gyro_x = IMUdata[3]
		self.gyro_y = IMUdata[4]
		self.gyro_z = IMUdata[5]
	def __str__(self):
		return "#### IFRAME DATA ####\n\
				--- ACCELEROMETER ---\n\
				x-dir: %i m/s^2\n\
				y-dir: %i m/s^2\n\
				z-dir: %i m/s^2\n\
				--- GYROSCOPE ---\n\
				x-dir: %i dps\n\
				y-dir: %i dps\n\
				z-dir: %i dps\n\
				#####################\n" % (self.accel_x,self.accel_y,self.accel_z,self.gyro_x,self.gyro_y, self.gyro_z)
	def convert_accel(ax,ay,az):
		accel_vals = [ax,ay,az]
		new_vals =[val * 9.80665 for val in accel_vals]
		return new_vals
		###returns as m/s^2###
	
	def get_imu_data():
		#############################
		# Strings for debug
		#############################
		imu_devs   = ["ACCELEROMETER","GYROSCOPE"]
		imu_labels = ["x-dir","y-dir","z-dir"]
		imu_units  = ["g","g","g","dps","dps","dps"]
		if not start_bool:
			print("IMU NOT SET UP CORRECTLY")
			return None
		try:
			ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
			ax,ay,az = convert_accel(ax, ay, az)
			#mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
		except:
		    return 
		print(50*"-")
		imu_values = [ax,ay,az,wx,wy,wz]
		for imu_ii, imu_val in enumerate(imu_values):
		    if imu_ii%3==0: #enumerate data for three directions then print next sensor
		        print(20*"_"+"\n"+imu_devs[int(imu_ii/3)]) # print sensor header
		    #
		    ###############
		    # Print Data
		    ###############
		    #
		    print("{0}: {1:3.2f} {2}".format(imu_labels[imu_ii%3],imu_val,imu_units[imu_ii]))
		return imu_values

t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from mpu9250_i2c import *
        start_bool = True # True for forthcoming loop
        break
    except:
    	continue

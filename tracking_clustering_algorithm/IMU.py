class iframe:
	def __init__(self, IMUdata):
		self.accel_x = IMUdata[0]
		self.accel_y = IMUdata[1]
		self.accel_z = IMUdata[2]
		self.gyro_x = IMUdata[3]
		self.gyro_y = IMUdata[4]
		self.gyro_z = IMUdata[5]

import IMU
class client():
	def __init__(self, IMU_data):
		self.x = -1
		self.y = -1
		self.x_velocity= -1
		self.y_velocity = -1
		self.x_estimate = -1
		self.y_estimate = -1
		self.imuFrame = IMU_data

class rframe():
	
	def __init__(self, frame_num, data, clients, prev_frame):
		self.frame_num = frame_num
		self.data = data #csv data, might need to be converted to pandas object
		self.clients = clients # array of clients with IMU data
		self.prev_frame = prev_frame

	def getCluster(self):
		### preform DBscan, eliminate noise ###
	def routerID(self):
		client_1_imu_data = clients[0].imuFrame
		client_2_imu_data = clients[1].imuFrame
		###find the router###
	def corePoint(self, client)
		client.x, client.y = 0 # core point center
		## identify center x,y)
		## give it a radius
		return client

	def kalmanFilter(self)
		client_1_imu_data = clients[0].imuFrame
		client_2_imu_data = clients[1].imuFrame
		x,y = 0 
		# estimate x,y leveraging imu data

		return clients #returns array of clients with estiamted positions.

	def getExtimate
		#logic before kalman filter
		final_clients = kalmanFilter()
		return final_clients #final clients is array of clients with all member variables finalized.


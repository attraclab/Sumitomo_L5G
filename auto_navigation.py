
import rospy
import rospkg
import rosparam
import yaml
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray, UInt8, Bool, Int8, Float32, Int8MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time
from simple_pid import PID
import time
import nav_params as nav
import sys
import argparse

python_version = int(sys.version_info[0])

if int(python_version) == 3:
	import importlib

class AutoNav:

	def __init__(self, sim):

		rospy.init_node("autonomous_drive_node", anonymous=True)

		self.got_scan = False

		self.reload_parameters()

		#######################
		### Scan parameters ###
		#######################
		self.scan_ready = False
		self.angle_list = None
		self.range_list = None
		self.got_first_scan = False

		self.left_max_index = 0
		self.left_wf_max_dist = 1.0
		self.prev_left_wf_max_dist = self.left_wf_max_dist 
		self.left_wf_perpen_dist = 1.0
		self.prev_left_wf_perpen = self.left_wf_perpen_dist
		self.left_wf_min_dist = 1.0
		self.prev_left_wf_min_dist = self.left_wf_min_dist
		self.no_left_dist = False

		self.right_max_index = 0
		self.right_wf_max_dist = 1.0
		self.prev_right_wf_max_dist = self.right_wf_max_dist 
		self.right_wf_perpen_dist = 1.0
		self.prev_right_wf_perpen = self.right_wf_perpen_dist
		self.right_wf_min_dist = 1.0
		self.prev_right_wf_min_dist = self.right_wf_min_dist
		self.no_right_dist = False

		self.front_stop_min_dist = 100.0
		self.prev_front_stop_min_dist = self.front_stop_min_dist
		self.no_front_dist = False

		###########
		### Nav ###
		###########
		self.vx = 0.0
		self.wz = 0.0
		self.nav_step = 1
		self.mission_done = False
		self.right_wf_flag = True # True
		self.nav_enable = False

		self.button_list = [0,0,0,0]

		#############
		### JMOAB ###
		#############
		self.ahrs_ready = False
		self.atcart_ready = False

		self.cart_mode = 1
		self.prev_cart_mode = self.cart_mode
		self.ch7 = 400
		self.prev_ch7 = self.ch7

		self.roll = 0.0
		self.pitch = 0.0
		self.hdg = 0.0

		###########
		### PID ###
		###########
		### Wall following 
		self.pid_wf = PID(self.wf_p, self.wf_i, self.wf_d, setpoint=self.wf_setpoint)
		self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)
		self.pid_wf.sample_time = 0.001
		self.pid_wf.output_limits = (-100.0, 100.0)
		self.pid_wf.auto_mode = True

		self.pid_thresh = 0.02

		###############
		### Pub/Sub ###
		###############
		self.left_wf_scan_pub = rospy.Publisher("/nav/left_wf_scan", LaserScan, queue_size=1)
		self.left_wf_dist_max_pub = rospy.Publisher("/nav/left_wf_dist_max", Float32, queue_size=1)
		self.left_wf_dist_min_pub = rospy.Publisher("/nav/left_wf_dist_min", Float32, queue_size=1)

		self.right_wf_scan_pub = rospy.Publisher("/nav/right_wf_scan", LaserScan, queue_size=1)
		self.right_wf_dist_max_pub = rospy.Publisher("/nav/right_wf_dist_max", Float32, queue_size=1)
		self.right_wf_dist_min_pub = rospy.Publisher("/nav/right_wf_dist_min", Float32, queue_size=1)

		self.front_stop_scan_pub = rospy.Publisher("/nav/front_stop_scan", LaserScan, queue_size=1)
		self.front_stop_min_pub = rospy.Publisher("/nav/front_stop_min", Float32, queue_size=1)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		self.nav_wf_stat_pub = rospy.Publisher("/nav/wf_status", String, queue_size=1)
		self.nav_wf_stat_msg = String()

		if (sim == 1):
			# imu_topic_name = "/imu"
			# odom_topic_name = "/odom"
			self.lidar_frame_name = "lidar_link"
			print("Running in simulation mode")
			rospy.Subscriber("/imu", Imu, self.imu_callback)
		else:
		# 	imu_topic_name = "/zmoab/imu"
		# 	odom_topic_name = "/odometry/filtered"
			self.lidar_frame_name = "laser_frame"
			print("Running in actual robot")
			rospy.Subscriber("/jmoab/ahrs", Float32MultiArray, self.ahrs_callback)


		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/jmoab/sbus_rc_ch", Int16MultiArray, self.sbus_rc_callback)
		rospy.Subscriber("/jmoab/ahrs", Float32MultiArray, self.ahrs_callback)
		rospy.Subscriber("/jmoab/cart_mode", UInt8, self.atcart_mode_callback)

		rospy.Subscriber("/nav/enable", Bool, self.nav_enable_callback)
		rospy.Subscriber("/nav/control_button", Int8MultiArray, self.nav_control_button_callback)

		self.loop()

		rospy.spin()

	##############
	### Params ###
	##############
	def reload_parameters(self):

		if python_version == 2:
			reload(nav)
		elif python_version == 3:
			importlib.reload(nav)

		## Velocities
		self.vx_wall_follow = nav.vx_wall_follow
		self.wz_wall_follow = nav.wz_wall_follow

		self.vx_uturn = nav.vx_uturn
		self.wz_uturn = nav.wz_uturn

		self.vx_webrtc = nav.vx_webrtc
		self.wz_webrtc = nav.wz_webrtc

		## LaserScan
		self.left_wf_min_scan_ang = nav.left_wf_min_scan_ang
		self.left_wf_max_scan_ang = nav.left_wf_max_scan_ang

		self.right_wf_min_scan_ang = nav.right_wf_min_scan_ang
		self.right_wf_max_scan_ang = nav.right_wf_max_scan_ang

		self.front_min_scan_ang = nav.front_min_scan_ang
		self.front_max_scan_ang = nav.front_max_scan_ang
		self.front_stop_dist = nav.front_stop_dist

		## PID
		self.wf_p = nav.wf_p
		self.wf_i = nav.wf_i
		self.wf_d = nav.wf_d
		self.wf_setpoint = nav.wf_setpoint

		print("Reload parameters ")
		print("--- Velocities ---")
		print("vx_wall_follow   : {}".format(self.vx_wall_follow))
		print("wz_wall_follow   : {}".format(self.wz_wall_follow))
		print("vx_uturn         : {}".format(self.vx_uturn))
		print("wz_uturn         : {}".format(self.wz_uturn))
		print("vx_webrtc         : {}".format(self.vx_webrtc))
		print("wz_webrtc         : {}".format(self.wz_webrtc))
		print("--- LaserScan ---")
		print("left_wf_min_scan_ang  : {}".format(self.left_wf_min_scan_ang))
		print("left_wf_max_scan_ang  : {}".format(self.left_wf_max_scan_ang))
		print("right_wf_min_scan_ang : {}".format(self.right_wf_min_scan_ang))
		print("right_wf_max_scan_ang : {}".format(self.right_wf_max_scan_ang))
		print("front_wf_min_scan_ang : {}".format(self.front_min_scan_ang))
		print("front_wf_max_scan_ang : {}".format(self.front_max_scan_ang))
		print("front_stop_dist       : {}".format(self.front_stop_dist))
		print("--- Wall-follow PID ---")
		print("wf_p             : {}".format(self.wf_p))
		print("wf_i             : {}".format(self.wf_i))
		print("wf_d             : {}".format(self.wf_d))
		print("wf_setpoint      : {}".format(self.wf_setpoint))

		if self.got_scan:
			self.update_lidar_index()

	def update_lidar_index(self):
		## left wall-follow
		self.left_wf_first_idx = self.lidarAng_to_lidarIdx(self.left_wf_min_scan_ang) 
		self.left_wf_last_idx = self.lidarAng_to_lidarIdx(self.left_wf_max_scan_ang)

		## right wall-follow
		self.right_wf_first_idx = self.lidarAng_to_lidarIdx(self.right_wf_min_scan_ang) 
		self.right_wf_last_idx = self.lidarAng_to_lidarIdx(self.right_wf_max_scan_ang)

		## front stop
		self.front_stop_first_idx = self.lidarAng_to_lidarIdx(self.front_min_scan_ang) 
		self.front_stop_last_idx = self.lidarAng_to_lidarIdx(self.front_max_scan_ang)	

	#####################
	### ROS callbacks ###
	#####################

	def scan_callback(self, msg):

		if not self.got_scan:
			self.scan_length = len(msg.ranges)
			self.got_scan = True
			self.update_lidar_index()
			print("scan length is {}".format(self.scan_length))
		else:
			
			### left wf ###
			left_wf_scan = LaserScan()
			left_wf_scan.header.stamp = rospy.Time.now()
			left_wf_scan.header.frame_id = self.lidar_frame_name #"laser_frame"
			left_wf_scan.time_increment = msg.time_increment
			left_wf_scan.angle_increment = msg.angle_increment
			left_wf_scan.angle_min = np.radians(self.left_wf_min_scan_ang)
			left_wf_scan.angle_max = np.radians(self.left_wf_max_scan_ang)
			left_wf_scan.scan_time = msg.scan_time
			left_wf_scan.range_min = msg.range_min
			left_wf_scan.range_max = msg.range_max
			left_wf_scan.ranges = msg.ranges[self.left_wf_first_idx:self.left_wf_last_idx]
			left_wf_scan.intensities = msg.intensities[self.left_wf_first_idx:self.left_wf_last_idx]

			left_wf_angle_list = np.arange((left_wf_scan.angle_min), left_wf_scan.angle_max, left_wf_scan.angle_increment)
			self.left_wf_max_dist = max(self.replace_inf_with_zero(left_wf_scan.ranges))
			self.left_max_index = np.argmax(self.replace_inf_with_zero(left_wf_scan.ranges))
			self.left_wf_perpen_dist = self.left_wf_max_dist*np.cos(left_wf_scan.angle_min - left_wf_angle_list[self.left_max_index])
			self.prev_left_wf_perpen = self.left_wf_perpen_dist

			self.left_wf_min_dist, self.no_left_dist = self.remove_zero_from_array(left_wf_scan.ranges, self.prev_left_wf_min_dist)
			self.prev_left_wf_min_dist = self.left_wf_min_dist

			left_wf_dist_min_msg = Float32()
			left_wf_dist_min_msg.data = self.left_wf_min_dist
			self.left_wf_dist_min_pub.publish(left_wf_dist_min_msg)

			left_wf_dist_max_msg = Float32()
			left_wf_dist_max_msg.data = self.left_wf_perpen_dist 
			self.left_wf_dist_max_pub.publish(left_wf_dist_max_msg)

			### right wf ###
			right_wf_scan = LaserScan()
			right_wf_scan.header.stamp = rospy.Time.now()
			right_wf_scan.header.frame_id = self.lidar_frame_name #"laser_frame"
			right_wf_scan.time_increment = msg.time_increment
			right_wf_scan.angle_increment = msg.angle_increment
			right_wf_scan.angle_min = np.radians(self.right_wf_min_scan_ang)
			right_wf_scan.angle_max = np.radians(self.right_wf_max_scan_ang)
			right_wf_scan.scan_time = msg.scan_time
			right_wf_scan.range_min = msg.range_min
			right_wf_scan.range_max = msg.range_max
			right_wf_scan.ranges = msg.ranges[self.right_wf_first_idx:self.right_wf_last_idx]
			right_wf_scan.intensities = msg.intensities[self.right_wf_first_idx:self.right_wf_last_idx]

			right_wf_angle_list = np.arange((right_wf_scan.angle_min), right_wf_scan.angle_max, right_wf_scan.angle_increment)
			self.right_wf_max_dist = max(self.replace_inf_with_zero(right_wf_scan.ranges))
			self.right_max_index = np.argmax(self.replace_inf_with_zero(right_wf_scan.ranges))
			self.right_wf_perpen_dist = self.right_wf_max_dist*np.cos(right_wf_scan.angle_min - right_wf_angle_list[self.right_max_index])
			self.prev_right_wf_perpen = self.right_wf_perpen_dist

			self.right_wf_min_dist, self.no_right_dist  = self.remove_zero_from_array(right_wf_scan.ranges, self.prev_right_wf_min_dist)
			self.prev_right_wf_min_dist = self.right_wf_min_dist

			right_wf_dist_min_msg = Float32()
			right_wf_dist_min_msg.data = self.right_wf_min_dist
			self.right_wf_dist_min_pub.publish(right_wf_dist_min_msg)

			right_wf_dist_max_msg = Float32()
			right_wf_dist_max_msg.data = self.right_wf_perpen_dist 
			self.right_wf_dist_max_pub.publish(right_wf_dist_max_msg)

			### front stop ###
			front_stop_scan = LaserScan()
			front_stop_scan.header.stamp = rospy.Time.now()
			front_stop_scan.header.frame_id = self.lidar_frame_name #"laser_frame"
			front_stop_scan.time_increment = msg.time_increment
			front_stop_scan.angle_increment = msg.angle_increment
			front_stop_scan.angle_min = np.radians(self.front_min_scan_ang)
			front_stop_scan.angle_max = np.radians(self.front_max_scan_ang)
			front_stop_scan.scan_time = msg.scan_time
			front_stop_scan.range_min = msg.range_min
			front_stop_scan.range_max = msg.range_max
			front_stop_scan.ranges = msg.ranges[self.front_stop_first_idx:self.front_stop_last_idx]
			front_stop_scan.intensities = msg.intensities[self.front_stop_first_idx:self.front_stop_last_idx]

			self.front_stop_min_dist, self.no_front_dist = self.remove_zero_from_array(front_stop_scan.ranges, self.prev_front_stop_min_dist)
			self.prev_front_stop_min_dist = self.front_stop_min_dist

			front_stop_min_msg = Float32()
			front_stop_min_msg.data = self.front_stop_min_dist
			self.front_stop_min_pub.publish(front_stop_min_msg)


			self.left_wf_scan_pub.publish(left_wf_scan)
			self.right_wf_scan_pub.publish(right_wf_scan)
			self.front_stop_scan_pub.publish(front_stop_scan)

		self.scan_ready = True

	def atcart_mode_callback(self, msg):
		self.prev_cart_mode = self.cart_mode
		self.cart_mode = msg.data

		if (self.prev_cart_mode == 2) and (self.cart_mode == 0) and ((self.ch5 > 1500) and (self.prev_ch5 == self.ch5)):
			print("detected mode changed by itself, try forcing to auto again")
			cart_mode_cmd_msg = UInt8()
			cart_mode_cmd_msg.data = 2
			self.cart_mode_cmd_pub.publish(cart_mode_cmd_msg)

		self.atcart_ready = True

	def ahrs_callback(self, msg):

		self.roll = msg.data[0]
		self.pitch = msg.data[1]
		self.hdg = msg.data[2]

		self.ahrs_ready = True

	def imu_callback(self, msg):

		qw = msg.orientation.w
		qx = msg.orientation.x
		qy = msg.orientation.y
		qz = msg.orientation.z

		angles = euler_from_quaternion([qx, qy, qz, qw])
		self.roll = np.degrees(angles[0])
		self.pitch = np.degrees(angles[1])
		self.hdg = self.ConvertTo360Range(np.degrees(angles[2]))

		self.ahrs_ready = True

	def sbus_rc_callback(self, msg):
		
		self.prev_ch7 = self.ch7
		self.ch7 = msg.data[6]
		# self.prev_ch8 = self.ch8
		# self.ch8 = msg.data[7]

		# if (self.prev_ch7 != self.ch7) and (self.ch7 > 1500):
		# 	print("Update nav_params, and PID tuning")
		# 	self.reload_parameters()
		# 	self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)

		if (self.prev_ch7 != self.ch7):
			print("restart mission")
			self.allow_uturn_stamp = time.time()
			self.right_wf_flag = True
			self.mission_done = False
			if (self.ch7 > 1500):
				print("Enable PID control")
				self.pid_wf.auto_mode = True
				self.allow_uturn_stamp = time.time()
				
			else:
				print("Disable PID control")
				self.cmd_vel_publisher(0.0, 0.0)
				self.pid_wf.auto_mode = False
				self.nav_step = 1

	def cmd_vel_publisher(self, vx, wz):
		cmd_vel_msg = Twist()
		cmd_vel_msg.linear.x = vx
		cmd_vel_msg.angular.z = wz
		self.cmd_vel_pub.publish(cmd_vel_msg)

	def nav_enable_callback(self, msg):

		self.nav_enable = msg.data
		self.mission_done = False
		self.right_wf_flag = True

		if self.nav_enable == True:
			print("Enable PID control by WebRTC")
			self.pid_wf.auto_mode = True
			self.allow_uturn_stamp = time.time()

		else:
			self.cmd_vel_publisher(0.0, 0.0)
			print("Disable PID control by WebRTC")
			self.pid_wf.auto_mode = False
			self.nav_step = 1

	def nav_control_button_callback(self, msg):

		self.button_list = msg.data


	###################
	### Math helper ###
	###################
	def replace_inf_with_zero(self, a_list):
		np_array = np.asarray(a_list)
		np_array[np.isposinf(np_array)] = 0

		return np_array.tolist()

	def lidarAng_to_lidarIdx(self, ang):
		return int(self.map_with_limit(ang, -180.0, 180.0, 0.0, (self.scan_length-1)))

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	def remove_zero_from_array(self, range_list, prev_value):

		array = np.asarray(range_list)
		array = array[array !=0]

		if (len(array) == 0):
			min_value = prev_value
			no_value = True
		else:
			array = array[array !=0]
			min_value = np.min(array)
			no_value = False

		if np.isinf(min_value):
			# print("Got inf!")
			min_value = prev_value
			no_value = False

		return min_value, no_value

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg

	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target

		return diff_ang, sign

	############
	### Loop ###
	############
	def loop(self):

		rate = rospy.Rate(20)

		print("Start autonomous driving")

		wait_ready_stamp = time.time()
		disable_nav_stamp = time.time()
		travel_dist = 0.0
		hdg_diff = 0.0
		line_hdg_stamp = time.time()
		line_hdg_stop_period = 0.0
		wf_status = "waiting for enable"

		while not (self.scan_ready and self.atcart_ready and self.ahrs_ready):

			if (time.time() - wait_ready_stamp) > 1.0:
				print("Wait for all ready | scan_ready: {} | atcart_ready: {} | ahrs_ready: {}".format(\
				self.scan_ready, self.atcart_ready, self.ahrs_ready))

				wait_ready_stamp = time.time()

				self.allow_uturn_stamp = time.time()

			# time.sleep(1)
			rate.sleep()

		while not rospy.is_shutdown():

			## enable navigation ##
			if ((self.ch7 > 1500) or (self.nav_enable)) and (not self.mission_done):

				#############################
				### nav_step 1            ###
				### do wall-following PID ###
				#############################
				if self.nav_step == 1:

					self.allow_uturn_period = time.time() - self.allow_uturn_stamp
					front_trigger_cond = (self.allow_uturn_period > 10.0) and \
										(self.front_stop_min_dist < self.front_stop_dist) 

					if front_trigger_cond:
						self.nav_step = 2
						self.vx = 0.0
						self.wz = 0.0
						self.cmd_vel_publisher(0.0, 0.0)
						self.pid_wf.auto_mode = False
						last_wf_hdg = self.hdg
						print("f_stop: {:.2f} ".format(self.front_stop_min_dist))
						print("Finished wall-following, front-stop trig")
						wf_status = "front detected"
						time.sleep(2)


					#########################
					### do wall-following ###
					#########################
					else:

						self.vx = self.vx_wall_follow

						############################
						### right wall-following ###
						############################
						if self.right_wf_flag:

							if self.right_wf_min_dist < 1.2:
								self.pid_wf.setpoint = 1.0
							else:
								self.pid_wf.setpoint = self.wf_setpoint

							if abs(self.right_wf_perpen_dist - self.right_wf_min_dist) <= 0.5:
								pid_in = (self.right_wf_perpen_dist + self.right_wf_min_dist) / 2.0
							else:
								diff_pp_sp = abs(self.right_wf_perpen_dist - self.pid_wf.setpoint)
								diff_min_sp = abs(self.pid_wf.setpoint - self.right_wf_min_dist)
								if diff_pp_sp > diff_min_sp:
									pid_in = self.right_wf_min_dist
								else:
									pid_in = self.right_wf_perpen_dist
								

							output_pid_wf = self.pid_wf(pid_in)

							## if input pid is not too far from setpoint +- threshold
							## we just let it goes straight easily, no need to apply steering from PID control
							if (pid_in > (self.pid_wf.setpoint+self.pid_thresh)) and (not self.no_right_dist): #self.right_wf_min_dist
								# print("go right")
								self.wz = self.map_with_limit(output_pid_wf, -100.0, 0.0, -self.wz_wall_follow, 0.0)

							elif (pid_in < (self.pid_wf.setpoint-self.pid_thresh)) and (not self.no_right_dist): #self.right_wf_min_dist
								# print("go left")
								self.wz = self.map_with_limit(output_pid_wf, 0.0, 100.0, 0.0, self.wz_wall_follow)

							else:
								# print("go straight")
								self.wz = 0.0

							wf_status = "right wall-following"

						###########################
						### left wall-following ###
						###########################
						else:

							if self.left_wf_min_dist < 1.2:
								self.pid_wf.setpoint = 1.0
							else:
								self.pid_wf.setpoint = self.wf_setpoint

							if abs(self.left_wf_perpen_dist - self.left_wf_min_dist) <= 0.5:
								pid_in = (self.left_wf_perpen_dist + self.left_wf_min_dist) / 2.0
							else:
								diff_pp_sp = abs(self.left_wf_perpen_dist - self.pid_wf.setpoint)
								diff_min_sp = abs(self.pid_wf.setpoint - self.left_wf_min_dist)
								if diff_pp_sp > diff_min_sp:
									pid_in = self.left_wf_min_dist
								else:
									pid_in = self.left_wf_perpen_dist
								

							output_pid_wf = self.pid_wf(pid_in)

							## if input pid is not too far from setpoint +- threshold
							## we just let it goes straight easily, no need to apply steering from PID control
							if (pid_in > (self.pid_wf.setpoint+self.pid_thresh)) and (not self.no_left_dist): #self.right_wf_min_dist
								# print("go right")
								self.wz = self.map_with_limit(output_pid_wf, -100.0, 0.0, self.wz_wall_follow, 0.0)

							elif (pid_in < (self.pid_wf.setpoint-self.pid_thresh)) and (not self.no_left_dist): #self.right_wf_min_dist
								# print("go left")
								self.wz = self.map_with_limit(output_pid_wf, 0.0, 100.0, 0.0, -self.wz_wall_follow)

							else:
								# print("go straight")
								self.wz = 0.0

							wf_status = "left wall-following"

						print("nav_step: {:d} right_wf: {} l_max: {:.2f} l_min: {:.2f} r_max: {:.2f} r_min: {:.2f} pid_sp: {:.2f} pid_in: {:.2f} pid_out: {:.2f} vx: {:.2f} wz: {:.2f} front_stop: {:.2f}".format(\
							self.nav_step, self.right_wf_flag, \
							self.left_wf_perpen_dist, self.left_wf_min_dist,\
							self.right_wf_perpen_dist, self.right_wf_min_dist, \
							self.pid_wf.setpoint, pid_in, output_pid_wf, \
							self.vx, self.wz, self.front_stop_min_dist))

				##########################
				### nav_step 2         ###
				### front stop trigger ###
				### do u-turning       ###
				##########################
				elif self.nav_step == 2:

					hdg_diff, diff_sign = self.find_smallest_diff_ang(last_wf_hdg, self.hdg)
					hdg_diff = self.ConvertTo180Range(hdg_diff*(-diff_sign))

					if abs(hdg_diff) < 170.0:
						self.vx = self.vx_uturn
						self.wz = self.wz_uturn
					else:
						self.vx = 0.0
						self.wz = 0.0
						self.nav_step = 3

						self.cmd_vel_publisher(0.0, 0.0)
						print("Finished U-Turn")
						time.sleep(2)

					wf_status = "turning"

					print("nav_step: {:d} hdg: {:.2f} hdg_diff: {:.2f} vx: {:.2f} wz: {:.2f}".format(\
						self.nav_step, self.hdg, hdg_diff, self.vx, self.wz))


				#############################
				### nav_step 3            ###
				### checking mission done ###
				#############################
				elif self.nav_step == 3:

					if self.right_wf_flag == True:
						self.right_wf_flag = False
						self.pid_wf.auto_mode = True
						self.nav_step = 1
						self.allow_uturn_stamp = time.time()
						print("Change to left wall-following")
					else:
						self.mission_done = True
						self.nav_step = 1
						print("Done mission")
						wf_status = "mission done"

				self.cmd_vel_publisher(self.vx, self.wz)

			## disable navigation ##
			else:

				if (time.time() - disable_nav_stamp) > 2.0:
					print("Wait for navigation enable switch (ch7)")
					wf_status = "waiting for enable"
					disable_nav_stamp = time.time()

				output_pid_log = 0.0
				hdg_diff = 0.0
				self.allow_uturn_stamp = time.time()

				#############################
				### WebRTC remote control ###
				#############################
				if self.button_list[0] == 1:
					self.cmd_vel_publisher(self.vx_webrtc, 0.0)
				elif self.button_list[1] == 1:
					self.cmd_vel_publisher(0.0, self.wz_webrtc)
				elif self.button_list[2] == 1:
					self.cmd_vel_publisher(0.0, -self.wz_webrtc)
				elif self.button_list[3] == 1:
					self.cmd_vel_publisher(-self.vx_webrtc/2, 0.0)
				else:
					self.cmd_vel_publisher(0.0, 0.0)


			self.nav_wf_stat_msg.data = wf_status
			self.nav_wf_stat_pub.publish(self.nav_wf_stat_msg)


			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Wall-following Navigation node')
	parser.add_argument('--sim',
						help="put 1 for simulation mode, default is 0")

	args = parser.parse_args(rospy.myargv()[1:])
	sim = args.sim

	if (sim == None) or (int(sim) != 1):
		sim = 0
	else:
		sim = 1

	AutoNav(sim)
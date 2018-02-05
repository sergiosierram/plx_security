#!/usr/bin/python
import rospy
import numpy
import struct
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sys import stdin

class PlxSecurity():
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.secure_vel_topic = self.rospy.get_param("secure_vel_topic","/secure_cmd_vel")
		self.secure_mode_topic = self.rospy.get_param("secure_mode_topic","/secure_mode")
		self.lrf_top_topic = self.rospy.get_param("laser_top_topic", "/RosAria/S3Series_1_laserscan")
		self.lrf_floor_topic = self.rospy.get_param("sonar_env_topic", "/scan")
		self.sonar_topic = self.rospy.get_param("sonar_env_topic", "/RosAria/sonar")
		self.use_lrf_top = self.rospy.get_param("use_lrf_top", True)
		self.use_lrf_floor = self.rospy.get_param("use_lrf_top", False)
		self.use_sonar = self.rospy.get_param("use_sonar", True)
		self.security_rate = self.rospy.get_param("security_rate", 20)
		self.stop_distance = self.rospy.get_param("stop_distance", 0.3)	
		self.slow_distance = self.rospy.get_param("slow_distance", 0.6)
		self.slow_speed = self.rospy.get_param("slow_speed", 0.25)
		self.width_ratio = self.rospy.get_param("width_ratio", 1.25)
		self.stop_distance_sonar = self.rospy.get_param("stop_distance_sonar", 0.20)	
		self.slow_distance_sonar = self.rospy.get_param("slow_distance_sonar", 0.30)
		self.slow_speed_sonar = self.rospy.get_param("slow_speed_sonar", 0.15)
		'''Subscribers'''
		if self.use_lrf_top: 
			self.sub_lrf_top = self.rospy.Subscriber(self.lrf_top_topic, LaserScan, self.callback_lrf_top)
		if self.use_lrf_floor:
			self.sub_lrf_floor = self.rospy.Subscriber(self.lrf_floor_topic, LaserScan, self.callback_lrf_floor)
		if self.use_sonar:
			self.sub_sonar = self.rospy.Subscriber(self.sonar_topic, PointCloud, self.callback_sonar)
		'''Publishers'''
		self.pub_secure_vel = self.rospy.Publisher(self.secure_vel_topic, Twist, queue_size = 10)
		self.pub_secure_mode = self.rospy.Publisher(self.secure_mode_topic, Bool, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("plx_security", anonymous = True)
		self.rate = self.rospy.Rate(self.security_rate)
		self.sonar_offset_front_x = 0.331
		self.sonar_offset_back_x = -0.331
		self.change = False
		self.main_security()

	def callback_lrf_top(self, msg):
		pass
	
	def callback_lrf_floor(self, msg):
		pass
	
	def callback_sonar(self,msg):
		self.sonar_front_left_x = msg.points[0].x
		self.sonar_front_left_y = msg.points[0].y
		self.sonar_front_right_x = msg.points[3].x
		self.sonar_front_right_y = msg.points[3].y
		print(self.sonar_front_left_x-0.331, self.sonar_front_right_x-0.331)
		self.change = True
		return

	def main_security(self):
		while not self.rospy.is_shutdown():
			if self.change:
				#print(map(ord, self.sonar_pointcloud.data))
				pass
			self.rate.sleep()
		
if __name__ == '__main__':
	try:
		sec = PlxSecurity()
	except rospy.ROSInterruptException:
		pass

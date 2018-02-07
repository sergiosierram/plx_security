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
		self.insecure_vel_topic = self.rospy.get_param("insecure_vel_topic","/insecure_cmd_vel")
		self.insecure_mode_topic = self.rospy.get_param("insecure_mode_topic","/insecure_mode")
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
		self.stop_distance_sonar = self.rospy.get_param("stop_distance_sonar", 0.25)	
		self.slow_distance_sonar = self.rospy.get_param("slow_distance_sonar", 0.36)
		self.slow_speed_sonar = self.rospy.get_param("slow_speed_sonar", 0.1)
		'''Subscribers'''
		if self.use_lrf_top: 
			self.sub_lrf_top = self.rospy.Subscriber(self.lrf_top_topic, LaserScan, self.callback_lrf_top)
		if self.use_lrf_floor:
			self.sub_lrf_floor = self.rospy.Subscriber(self.lrf_floor_topic, LaserScan, self.callback_lrf_floor)
		if self.use_sonar:
			self.sub_sonar = self.rospy.Subscriber(self.sonar_topic, PointCloud, self.callback_sonar)
		'''Publishers'''
		self.pub_insecure_vel = self.rospy.Publisher(self.insecure_vel_topic, Twist, queue_size = 10)
		self.pub_insecure_mode = self.rospy.Publisher(self.insecure_mode_topic, Bool, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("plx_security", anonymous = True)
		self.rate = self.rospy.Rate(self.security_rate)
		self.sonar_offset_front_x = 0.331
		self.sonar_offset_back_x = -0.331
		self.change_sonar = False
		self.vel_insecure = Twist()
		self.main_security()

	def callback_lrf_top(self, msg):
		pass
	
	def callback_lrf_floor(self, msg):
		pass
	
	def callback_sonar(self,msg):
		self.sonar_front_left_x = float(msg.points[0].x)-0.331
		self.sonar_front_right_x = float(msg.points[3].x)-0.331
		self.change_sonar = True
		return

	def main_security(self):
		insecure = False
		dist = 0
		velmax = 0
		while not self.rospy.is_shutdown():
			rstop = min(self.slow_distance_sonar, self.stop_distance_sonar)
			rslow = max(self.slow_distance_sonar, self.stop_distance_sonar)
			if self.change_sonar:
				dist = min(self.sonar_front_left_x, self.sonar_front_right_x)
				if dist < rstop:
					velmax = 0
					insecure = True
				elif dist < rslow:
					velmax = self.slow_speed_sonar*(dist - rstop)/(rslow-rstop)
					insecure = True
				else:
					insecure = False
				print(insecure, dist, velmax)
			if insecure:
				self.pub_insecure_mode.publish(True)
				self.vel_insecure.linear.x = velmax
				self.pub_insecure_vel.publish(self.vel_insecure)
				self.change_sonar = False
			else:
				self.pub_insecure_mode.publish(False)
			self.rate.sleep()
		
if __name__ == '__main__':
	try:
		sec = PlxSecurity()
	except rospy.ROSInterruptException:
		pass

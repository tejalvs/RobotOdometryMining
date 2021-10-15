#!/usr/bin/env python3
import rospy, sys
#from corobot_common.msg import Pose
from sensor_msgs.msg import LaserScan,Image,CameraInfo
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2,isnan,sin,cos,dist
import numpy as np
import matplotlib.pyplot as plt
from collections import OrderedDict
import decimal

class Pose():
	def __init__(self):
	    	self.header = None
	    	self.x = 0
	    	self.y = 0
	    	self.theta = 0


def odom_to_pose(odom):
	"""Utility function to convert an Odometry message into a Pose message."""
	pose = Pose()
	pose.header = odom.header
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	qz = odom.pose.pose.orientation.z
	qw = odom.pose.pose.orientation.w
	pose.theta = atan2(2 * qw * qz, 1 - 2 * qz * qz)
	return pose

class mover():
	def __init__(self, t, v, omega):
		self.vel = v
		self.omega = omega
		self.tm = t
		self.vpub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.startt = None
		self.mspub = rospy.Publisher('/cmd_motor_state', MotorState, queue_size = 1,latch=True)
		self.mspub.publish(1)
		self.initial_position=None
		self.counter=0
		self.initial_range_points=None
		self.initial_dist_fr_wall=None

	def odom_callback(self, odom):
	    	#get the initial_position of the robot
		if self.counter==0:
			self.initial_position = odom_to_pose(odom)
			self.counter+=1
			init_data="x="+str(self.initial_position.x)+" y="+str(self.initial_position.y)+" theta="+str(self.initial_position.theta)
			rospy.loginfo("initial Pose information(x,y, theta, v, omega) %s",init_data)
		self.pose = odom_to_pose(odom)
		#calculate the distance travelled by the robot
		distance_travelled=self.get_distance(self.initial_position.x,self.initial_position.y,self.pose.x,self.pose.y)
		#log information
		logdata="x="+str(self.pose.x)+",y="+str(self.pose.y)+",theta="+str(self.pose.theta)+",velocity="+str(self.vel)+",omega="+str(self.omega)+",distance_travelled="+str(distance_travelled)
		rospy.loginfo("Odometry:Pose information  %s",logdata)

# 	def get_wall_coordinates(self,range_data,angle_min,angle_max,angle_increment):
# 		range_points={}
# 		for i in range(len(range_data)):
# 			if isnan(range_data[i]):
# 				continue
# 			else:
# 				alpha=angle_min+(i*angle_increment)
# 				x=(self.initial_position.x+cos(self.initial_position.theta+alpha))*range_data[i]
# 				y=(self.initial_position.y+sin(self.initial_position.theta+alpha))*range_data[i]
# 				range_points[alpha]=[x,y]
# 		return range_points
#
# 	def find_center_angle(self,key_store,angle_increment):
# 		min_val=float('inf')
# 		for value in key_store:
# 	    		if abs(value)< abs(min_val) :
# 	    			min_val=value
# 		return min_val
#
	def get_distance(self,p1x,p1y,p2x,p2y):
		result= ((((p2x-p1x )**2) + ((p2y-p1y)**2) )**0.5)
		return result

	def get_avg_distance(self,range_data,angle_min,angle_max,angle_increment):
		n=0
		total=0
		alpha=0
		for i in range(len(range_data)):
			if isnan(range_data[i]):
				continue
			else:
				alpha=(angle_min+(i*angle_increment))
				if abs(alpha) <= 0.0174533:
					total=total+range_data[i]
					n=n+1
		total=total/n
		return total

	def scan_callback(self, scan):
		range_data=scan.ranges
		angle_min=scan.angle_min
		angle_max=scan.angle_max
		angle_increment=scan.angle_increment
		if self.counter==1:
			self.initial_dist_fr_wall=self.get_avg_distance(range_data,angle_min,angle_max,angle_increment)
			log_data="Scan Data: inital wall diatance ="+str(self.initial_dist_fr_wall)
			rospy.loginfo(log_data)
			self.counter=self.counter+1
		else:
			delta=self.initial_dist_fr_wall-self.get_avg_distance(range_data,angle_min,angle_max,angle_increment)
			log_data="Scan Data: Distance Travelled="+str(delta)
			rospy.loginfo(log_data)


# 	def scan_callback(self, scan):
# 		range_data=scan.ranges
# 		angle_min=scan.angle_min
# 		angle_max=scan.angle_max
# 		angle_increment=scan.angle_increment
# 		init_center=None
# 		#get distance of the wall
# 		if self.counter==1:
# 			#get all co-ordinates
# 			self.initial_range_points=self.get_wall_coordinates(range_data,angle_min,angle_max,angle_increment)
# 			#find the central 0 degree  point
# 			center=self.initial_range_points[self.find_center_angle(self.initial_range_points.keys(),angle_increment)]
#             #calculate distance from wall
#             self.initial_dist_fr_wall=self.get_distance(self.initial_position.x,self.initial_position.y,center[0],center[1])
# 			self.counter+=1
# 		else:
# 		    delta=initial_dist_fr_wall-range_data[angle_max//angle_increment]
# 		    alpha=angle_min+(i*angle_increment)
# 		    x=(self.initial_position.x+cos(self.initial_position.theta+alpha))*range_data[i]
#             y=(self.initial_position.y+sin(self.initial_position.theta+alpha))*range_data[i]
#  			range_points=self.get_range_coordinates(range_data,angle_min,angle_max,angle_increment)
# 			center=range_points[self.find_point(range_points.keys(),angle_increment)]
# 			d=self.get_distance(self.initial_position.x,self.initial_position.y,center[0],center[1])
# 			current_d=self.initial_dist_fr_wall-d
# 			t=current_d/d
# 			x_val=(1-t)*self.initial_position.x+t*


	def timer_callback(self, event):
		if self.startt is None:
		    self.startt = rospy.get_rostime()
		dt = rospy.get_rostime() - self.startt
		if dt.secs >= self.tm:
		    self.vpub.publish(Twist())
		    sys.exit()
		# if the velocity is constant, can make this in constructor
		# but we may want more complex things, so...
		tw = Twist()
		tw.linear.x = self.vel
		tw.angular.z = self.omega
		self.vpub.publish(tw)


def main():
	rospy.init_node("mover")
	# arguments are time, fwd vel, angular vel
	t = float(sys.argv[1])
	v = float(sys.argv[2])
	w = float(sys.argv[3])
	mv = mover(t,v,w)
	rospy.Subscriber("/pose", Odometry, mv.odom_callback)
	rospy.Subscriber("/scan", LaserScan, mv.scan_callback)
	rospy.Timer(rospy.Duration(0.1), mv.timer_callback)
	rospy.spin()

main()
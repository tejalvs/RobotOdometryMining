#!/usr/bin/env python3
import rospy, sys
#from corobot_common.msg import Pose
from sensor_msgs.msg import LaserScan,Image,CameraInfo
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2,isnan,sin,cos
import numpy as np
import matplotlib.pyplot as plt
from collections import OrderedDict

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


	def odom_callback(self, odom):
		if self.counter==0:
			self.initial_position = odom_to_pose(odom)
			self.counter+=1
			init_data="x="+str(self.initial_position.x)+" y="+str(self.initial_position.y)+" theta="+str(self.initial_position.theta)
			rospy.loginfo("initial Pose information(x,y, theta, v, omega) %s",init_data)
		self.pose = odom_to_pose(odom)
		logdata="x="+str(self.pose.x)+" y="+str(self.pose.y)+" theta="+str(self.pose.theta)+"velocity="+str(self.vel)+" omega="+str(self.omega)
		rospy.loginfo("Pose information(x,y, theta, v, omega) %s",logdata)

	def get_range_coordinates(self,range_data,angle_min,angle_max,angle_increment,delta):
		range_points={}
		for i in range(len(range_data)):
			if isnan(range_data[i]):
				continue
			else:
				alpha=angle_min+(i*angle_increment)
				x=(self.initial_position.x+cos(self.initial_position.theta+alpha))*range_data[i]
				y=(self.initial_position.y+sin(self.initial_position.theta+alpha))*range_data[i]
				range_points[alpha]=[x,y]
	    return range_points

    def find_point(self,key_store):
        for value in key_store:
            if value in range(-0.0174533,0.0174533)
                return value
	def scan_callback(self, scan):
		range_data=scan.ranges
		angle_min=scan.angle_min
		angle_max=scan.angle_max
		angle_increment=scan.angle_increment
		if self.counter==1:
			 self.initial_range_points=self.get_range_coordinates(range_data,angle_min,angle_max,angle_increment,0)
			 center=self.initial_range_points[self.find_point(self.initial_range_points.keys()]
             rospy.loginfo(center)
# 			 line_x=np.array([self.initial_position.x,x[len(x)//2]])
# 			 line_y=np.array([self.initial_position.y,y[len(y)//2]])
# 			 plt.xlim((int(min(x)-10)), (int(max(x)+10)))
# 			 plt.ylim((int(min(y)-10)), (int(max(y)+10)))
# 			 plt.scatter(x,y,linestyle='--', marker='o', color='b')
# 			 plt.scatter((self.initial_position.x),(self.initial_position.y),10,color="red")
# 			 rospy.loginfo(str(x_max)+","+str(y_max))
# 			 plt.scatter((x[len(x)//2]),y[len(y)//2],10,color="red")
# 			 plt.plot(line_x,line_y)
# 			 plt.savefig('/home/driver/catkin_ws/src/mover/src/initial_position.png')
# 			 plt.show()
			 self.counter+=1
		else:
		    range_points=self.get_range_coordinates(range_data,angle_min,angle_max,angle_increment,0)



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


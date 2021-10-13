#!/usr/bin/env python3
import rospy, sys
#from corobot_common.msg import Pose
from sensor_msgs.msg import LaserScan,Image,CameraInfo
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2,isnan,sin,cos
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
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
		self.initial_range=None


	def odom_callback(self, odom):
		if self.counter==0:
			self.initial_position = odom_to_pose(odom)
			self.counter+=1
			ogdata="x="+str(self.initial_position.x)+" y="+str(self.initial_position.y)+" theta="+str(self.initial_position.theta)
			#rospy.loginfo("initial Pose information(x,y, theta, v, omega) %s",ogdata)
		self.pose = odom_to_pose(odom)
		logdata="x="+str(self.pose.x)+" y="+str(self.pose.y)+" theta="+str(self.pose.theta)+"velocity="+str(self.vel)+" omega="+str(self.omega)
		#rospy.loginfo("Pose information(x,y, theta, v, omega) %s",logdata)

	def get_range_coordinates(self,range_data,angle_min,angle_max,angle_increment):
		x=[]
		y=[]
		x_max=0
		y_max=0
		angle_max=None
		max_value=float("-inf")
		log_data=None
		for i in range(len(range_data)):
			if isnan(range_data[i]):
				#rospy.loginfo("True")
				continue
			else:
				alpha=angle_min+(i*angle_increment)
				x_new=(self.initial_position.x+cos(self.initial_position.theta+alpha))
				y_new=(self.initial_position.y+sin(self.initial_position.theta+alpha))
				x.append(x_new*100)
				y.append(y_new*100)
		x.append(self.initial_position.x*100)
		y.append(self.initial_position.y*100)
		log_data= "x="+str(x_max)+" y="+str(y_max)+" theta="+str(angle_max)
		return x,y,x_max,y_max


	def scan_callback(self, scan):
		range_data=scan.ranges
		angle_min=scan.angle_min
		angle_max=scan.angle_max
		angle_increment=scan.angle_increment
		x_max=0
		y_max=0
		#x,y=self.get_range_coordinates(range_data,angle_min,angle_max,angle_increment)
		if self.counter==1:
			 x,y,x_max,y_max=self.get_range_coordinates(range_data,angle_min,angle_max,angle_increment)
			 line_x=np.array([self.initial_position.x,x[len(x)//2]])
			 line_y=np.array([self.initial_position.y,y[len(y)//2]])
			 plt.xlim((int(min(x)-10)), (int(max(x)+10)))
			 plt.ylim((int(min(y)-10)), (int(max(y)+10)))
			 plt.scatter(x,y,linestyle='--', marker='o', color='b')
			 plt.scatter((self.initial_position.x),(self.initial_position.y),10,color="red")
			 rospy.loginfo(str(x_max)+","+str(y_max))
			 plt.scatter((x[len(x)//2]),y[len(y)//2],10,color="red")
			 plt.plot(line_x,line_y)
			 plt.savefig('/home/driver/catkin_ws/src/mover/src/initial_position.png')
			 plt.show()
			 self.counter+=1


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
	#  	rospy.Subscriber("/camera/rgb/image_color", Image, mv.image_callback)
	rospy.Timer(rospy.Duration(0.1), mv.timer_callback)
	rospy.spin()

main()


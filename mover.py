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
	

	def odom_callback(self, odom):
		self.pose = odom_to_pose(odom)
		logdata="x="+str(self.pose.x)+" y="+str(self.pose.y)+" theta="+str(self.pose.theta)+" velocity="+str(self.vel)+" omega="+str(self.omega)
		rospy.loginfo("Pose information(x,y, theta, v, omega) %s",logdata)
		#rospy.loginfo(self.pose.x, self.pose.y,self.pose.theta)

	def scan_callback(self, scan):
		range_data=scan.ranges
		angle_min=scan.angle_min
		angle_max=scan.angle_max
		angle_increment=scan.angle_increment
		x=[]
		y=[]
		x_init=0
		y_init=0
		theta=0
		for i in range(len(range_data)):
			if isnan(range_data[i]):
				#rospy.loginfo("True")
				continue
			else:
				alpha=angle_min+(i*angle_increment)
				x_new=(x_init+cos(theta+alpha))*100
				y_new=(y_init+sin(theta+alpha))*100
				x.append(x_new)
				y.append(y_new)
		x.append(0)
		y.append(0)
		plt.xlim(-100, 110)
		plt.ylim(-100, 110)
		plt.scatter(x,y)
		plt.show()
		sys.exit()
		

          
          
          
	def image_callback(self, msg):
		bridge=CvBridge()
		rospy.loginfo("image information %s")
		try:
			cv_img=bridge.imgmsg_to_cv2(msg,"bgr8")
			cv_img=(255-cv_img)
			thresh=cv2.Canny(cv_img,5,150,5)
			lines=cv2.HoughLinesP(thresh,2,np.pi/180,100,100,1)
			rospy.loginfo(lines)
			for i in range(lines.shape[0]):
				for x1,y1,x2,y2 in lines[i]:
					cv2.line(cv_img,(x1,y1),(x2,y2),(0,255,0),2)
		except CvBridgeError:
			print(e)
		cv2.imwrite("image.jpeg",lines)
		cv2.waitKey(3)	
			
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
		
	def get_init_position(self):
		data_odom=None
		while data_odom is None:
			try :
				data_odom=rospy.wait_for_messages("/odom",Odometry, timeout=1)
			except:
				rospy.loginfo("Current odom not ready yet, retrying  for setting up init position")
		self.start_position=Point()
		self.start_position=data_odom.pose.pose.position.x
		self.start_position=data_odom.pose.pose.position.y
		print()
		
			



def main():
     rospy.init_node("mover")
     # arguments are time, fwd vel, angular vel
     t = float(sys.argv[1])
     v = float(sys.argv[2])
     w = float(sys.argv[3])
     mv = mover(t,v,w)
     rospy.Subscriber("/pose", Odometry, mv.odom_callback)
     rospy.Subscriber("/scan", LaserScan, mv.scan_callback)
     #rospy.Subscriber("/camera/rgb/image_color", Image, mv.image_callback)
     #rospy.Timer(rospy.Duration(0.1), mv.timer_callback)
     rospy.spin()
     
main()



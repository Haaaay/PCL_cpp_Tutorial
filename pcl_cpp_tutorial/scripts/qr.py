#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers

qr_z_pos=0.0
qr_x_pos=0.0
sign_flag=0
cmd_vel=Twist()
msg=AlvarMarkers()

def qr(msg):
	global cmd_vel
	global qr_z_pos
	global qr_x_pos
	global sign_flag

	cmd_vel.linear.x=5.0
	qr_z_pos=msg.pose.pose.position.z
	qr_x_pos=msg.pose.pose.position.x
	qr_distance=math.sqrt(math.pow(qr_z_pos,2)+math.pow(qr_x_pos,2))
	rate=rospy.Rate(30)

	if len(msg.markers)!=0 and 0<qr_distance<0.5:	
		rospy.loginfo("QR Code Distance : %f", qr_distance)
		if msg.markers.id==0 and sign_flag==0:
			rospy.loginfo("---Sleep---")
			hello_str="Warning"
			rospy.loginfo(hello_str)
			time.sleep(3)
			sign_flag+=1
		elif msg.markers.id==0 and sign_flag>0:
			hello_str="Warning"
			rospy.loginfo(hello_str)
			cmd_vel.linear.x=5.0
		elif msg.markers.id==1:
			hello_str="Warning"
			rospy.loginfo(hello_str)
			cmd_vel.linear.x=1.0
		else:
			hello_str="safe"
			rospy.loginfo(hello_str)
	pub.publish(hello_str)
	pub1.publish(cmd_vel)
	rate.sleep()

rospy.init_node('wego_node')
sub1=rospy.Subscriber('/camera/ar_track_alvar', AlvarMarkers, callback=qr)
pub=rospy.Publisher('/limo/lidar_warning', String, queue_size=10)
pub1=rospy.Publisher('/limo/ldar_cmd_vel', Twist, queue_size=10)
rospy.spin()



#!/usr/bin/env python

import rospy
import math
import time
from sensor_msgs.msg import LaserScan

start_degree=-45
end_degree=45
sum=0
cmd_vel=5.0
lidar_obstacle_range=0.3
traffic_sign_flag=0
count_slow=0
start_time=0.0
end_time=0.0

def scanCallback(data):
	#rospy.loginfo("range_min : %f", data.range_min)
	global sum
	sum=0
	global traffic_sign_flag
	global cmd_vel
	global start_time
	global end_time
	for count in range(len(data.ranges)): #int(360/(data.angle_increment*(180/math.pi)))
		degree=(data.angle_min+data.angle_increment*count)*(180/math.pi)
		#rospy.loginfo("%f\n", degree);
		#rospy.loginfo("%d  |  %f\n", count, data.ranges[count])
		if (start_degree<degree<end_degree) and 0.1<data.ranges[count]<=lidar_obstacle_range:
			sum+=1
			#rospy.loginfo("%f	|	%f\n", degree, data.ranges[count])
	if sum>10:
		if traffic_sign_flag==0:
			rospy.loginfo("---Sleep---")
			time.sleep(3)
			start_time=time.time()
			rospy.loginfo("start_time : %d", start_time)
			traffic_sign_flag+=1
		else:
			cmd_vel=0.5
	end_time=time.time()
	if end_time-start_time>=5:
		cmd_vel=5.0
		traffic_sign_flag=0

	rospy.loginfo("Sum : %d", sum)
	rospy.loginfo("Command Velocity : %f\n", cmd_vel)
	

rospy.init_node("Lidar_Homework", anonymous=True)
sub=rospy.Subscriber("/scan", LaserScan, scanCallback)

rospy.spin()

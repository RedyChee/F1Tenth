#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Header
from cheeyengsung_lab1.msg import scan_range

class LidarProcess:
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.range_max_pub = rospy.Publisher("/farthest_point", Float64, queue_size = 10)
		self.range_min_pub = rospy.Publisher("/closest_point", Float64, queue_size = 10)
		self.range_pub = rospy.Publisher("/scan_range", scan_range, queue_size = 10)
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)

	def callback(self, scan_data):
		try: 
			filter_data = [val for val in scan_data.ranges if not np.isinf(val) and not np.isnan(val)] 
			f_max = Float64()
			f_min = Float64()
			f_max.data = max(filter_data)
			f_min.data = min(filter_data)
			self.range_max_pub.publish(f_max)
			self.range_min_pub.publish(f_min)
			f_range = scan_range()
			f_range.header = scan_data.header
			f_range.range_max = max(filter_data)
			f_range.range_min = min(filter_data)
			self.range_pub.publish(f_range)
		except rospy.ROSException as e:
#			rospy.logerr(e)
			pass
			
if __name__ == '__main__':
	try:
		rospy.init_node('lidar_processor', anonymous=True)
		ld = LidarProcess()
		rospy.spin()
	except rospy.ROSException as e:
#		rospy.logerr(e)
		pass

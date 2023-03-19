#! /usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs import *
import numpy as np
from zed_interfaces.msg import ObjectsStamped

# Date received
def callback(data):
	raw_point = np.empty((8, 3))
	max_len = len(data.objects)
	for i in range(max_len):
		for j in range(0, 8):
			# i : object number
			# j : keypoints
			raw_point[j, 0] = data.objects[i].bounding_box_3d.corners[j].kp[0]
			raw_point[j, 1] = data.objects[i].bounding_box_3d.corners[j].kp[1]
			raw_point[j, 2] = data.objects[i].bounding_box_3d.corners[j].kp[2]
	print("raw_point: \n", raw_point)

def zed_subscriber():
	# objectstamped msg type !!subscribe!!
	rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, callback)

def marker_publisher():
	# Marker publisher 정의
	box_publisher = rospy.Publisher('bounding_boxes', Marker, queue_size=10)

	line_list = [Marker()] * 12

	for l in range(len(line_list)):
		line_list[l] = Marker()
		line_list[l].action = Marker().ADD
		line_list[l].header.frame_id = "map"
		line_list[l].ns = "line_" + str(l)
		line_list[l].pose.orientation.w = 1.0
		line_list[l].type = Marker().LINE_STRIP
		line_list[l].id = l
		line_list[l].scale.x = 0.1
		line_list[l].color.b = 1.0
		line_list[l].color.g = 1.0
		line_list[l].color.a = 1.0

	cube1 = np.array([[0, 0, 5], [0, 4, 5], [3, 4, 5], [3, 0, 5], [0, 0, 5],
					  [0, 0, 0], [0, 4, 0], [3, 4, 0], [3, 0, 0], [0, 0, 0]])

	for i in range(1, 5):
		p = Point()
		d = Point()
		d.x = cube1[i-1, 0]
		d.y = cube1[i-1, 1]
		d.z = cube1[i-1, 2]
		p.x = cube1[i, 0]
		p.y = cube1[i, 1]
		p.z = cube1[i, 2]
		line_list[i-1].points.append(p)
		line_list[i-1].points.append(d)
		box_publisher.publish(line_list[i-1])

	for i in range(6, 10):
		p = Point()
		d = Point()
		d.x = cube1[i-1, 0]
		d.y = cube1[i-1, 1]
		d.z = cube1[i-1, 2]
		p.x = cube1[i, 0]
		p.y = cube1[i, 1]
		p.z = cube1[i, 2]
		line_list[i-2].points.append(d)
		line_list[i-2].points.append(p)
		box_publisher.publish(line_list[i-2])

	# 다른 포인트를 써서 해보자.

	for i in range(5):
		p = Point()
		d = Point()
		d.x = cube1[i, 0]
		d.y = cube1[i, 1]
		d.z = cube1[i, 2]
		p.x = cube1[i+5, 0]
		p.y = cube1[i+5, 1]
		p.z = cube1[i+5, 2]
		# 8, 9, 10, 11
		line_list[i+7].points.append(d)
		line_list[i+7].points.append(p)
		box_publisher.publish(line_list[i+7])

if __name__ == '__main__':
	while not rospy.is_shutdown():
		rospy.init_node('bounding_box_publisher')
		# zed_subscriber()
		marker_publisher()

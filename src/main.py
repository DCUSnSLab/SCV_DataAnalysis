#! /usr/bin/env python3
import rospy
import time
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs import *
import numpy as np
import pyzed.sl as sl
import cv2
from zed_interfaces.msg import Object
import yaml
from rosbag.bag import Bag
import rosbag
import rospy

def subscriber():
	topics = "/zed2/zed_node/obj_det/objects"
	bag = rosbag.Bag('230209_DCU.bag')
	for topic, msg in bag.read_messages(topics=[topics, Object]):
		print(msg)
	bag.close()

	rospy.init_node('bounding_box_subscriber')

	def callback(data):
		print("%s", data.data)
		rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	rospy.Subscriber("/zed2/zed_node/obj_det/objects", Object, callback)
	# spin()은 노드가 중지될 때까지 파이썬이 종료되지 않도록 합니다.
	rospy.spin()

def publisher():
	# Initialize ROS node and publisher
	rospy.init_node('bounding_box_publisher')
	publisher = rospy.Publisher('bounding_boxes', Marker, queue_size=10)
	line_list = [Marker(), Marker(), Marker()]
	# Create a Marker message to represent a bounding box
	points = line_strip = Marker()
	points.header.frame_id = line_strip.header.frame_id = "map"
	points.ns = "points"
	line_strip.ns = "lines"
	# action : 마커 ADD, DELETE 등 지정
	points.action = line_strip.action = Marker().ADD
	points.pose.orientation.w = line_strip.pose.orientation.w = 1.0

	# x, y, z 변수바꾸기

	for i in range(0, 3):
		line_list[i] = Marker()
		line_list[i].header.frame_id = "map"
		line_list[i].ns = "lines"
		line_list[i].action = Marker().ADD
		line_list[i].pose.orientation.w = 1.0
		line_list[i].type = Marker().LINE_STRIP
		line_list[i].id = i
		line_list[i].scale.x = 0.1
		line_list[i].color.g = 1.0
		line_list[i].color.b = 1.0
		line_list[i].color.a = 1.0

	points.type = Marker().POINTS
	line_strip.type = Marker().LINE_STRIP

	# id
	points.id = 4
	line_strip.id = 5

	# vector
	points.scale.x = 0.1
	points.scale.y = 0.1
	line_strip.scale.x = 0.1

	# color
	points.color.g = 1.0
	points.color.a = 1.0
	line_strip.color.b = 1.0
	line_strip.color.a = 1.0

	# box point number
	# 1 - 2 - 3 - 4
	cube1 = [[0, 0, 5], [0, 4, 5], [3, 4, 5], [3, 0, 5], [0, 0, 5]]
	# 5 - 6 - 7 - 8
	cube2 = [[0, 0, 0], [0, 4, 0], [3, 4, 0], [3, 0, 0], [0, 0, 0]]

	for i in range(len(cube1)):
		p = Point()
		p.x = cube1[i][0]
		p.y = cube1[i][1]
		p.z = cube1[i][2]
		points.points.append(p)
		line_strip.points.append(p)

	for i in range(len(cube2)):
		p = Point()
		p.x = cube2[i][0]
		p.y = cube2[i][1]
		p.z = cube2[i][2]
		points.points.append(p)
		line_strip.points.append(p)

	for i in range(1, 4):
		p = Point()
		d = Point()

		p.x = cube1[i][0]
		p.y = cube1[i][1]
		p.z = cube1[i][2]

		d.x = cube2[i][0]
		d.y = cube2[i][1]
		d.z = cube2[i][2]

		# 0 -> 1 -> 2
		line_list[i-1].points.append(p)
		line_list[i-1].points.append(d)
		publisher.publish(line_list[i-1])

	rospy.sleep(5)
	publisher.publish(line_strip)
	publisher.publish(points)


if __name__ == '__main__':
	while not rospy.is_shutdown():
		# publisher()
		subscriber()
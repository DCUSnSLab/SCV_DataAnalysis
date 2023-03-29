#! /usr/bin/env python3
import rospy
import std_msgs.msg
import sensor_msgs.msg
import numpy as np
import time

# rospy.Rate : 지정된 속도로 루프에서 잠을 잘 수 있는 편의 클래스
# rospy.Timer : 지정된 속도로 콜백을 호출하기 위한 편의 클래스

# ROS Duration으로 시차 모니터링
pc2_pub = rospy.Publisher('/sampling_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)

# def callback(data):
#     rate = rospy.Rate(50)  # ROS Rate at 5Hz
#
# if __name__ == '__main__':
#     rospy.init_node("Sampling_pc2")
#     rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, callback)
#     rospy.spin()



# 처음 들어온 데이터가 계속 반복됨!!!!!!!
def callback(data):
    rate = rospy.Rate(0.5)  # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        pc2_pub.publish(data)
        print(rate)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("Sampling_pc2")
    rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, callback)
    rospy.spin()

# compressedImage
# Pointcloud2
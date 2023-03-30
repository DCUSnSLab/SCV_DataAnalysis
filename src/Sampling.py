#! /usr/bin/env python3
import rospy
import std_msgs.msg
import sensor_msgs.msg
import time

# rospy.Rate : 지정된 속도로 루프에서 잠을 잘 수 있는 편의 클래스
# rospy.Timer : 지정된 속도로 콜백을 호출하기 위한 편의 클래스

# ROS Duration으로 시차 모니터링
pc2_pub = rospy.Publisher('/sampling_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
ci_pub = rospy.Publisher('/sampling_ci', sensor_msgs.msg.CompressedImage, queue_size=10)

def PC_callback(data):
    rate = rospy.Rate(0.5)  # ROS Rate at 5Hz
    pc2_pub.publish(data)
    rate.sleep()

def CI_callback(data):
    rate = rospy.Rate(0.5)  # ROS Rate at 5Hz
    ci_pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node("Sampling_pc2")
        # Pointcloud2
        rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, PC_callback)
        # compressedImage
        rospy.Subscriber('/zed2/zed_node/left/image_rect_color/compressed', sensor_msgs.msg.CompressedImage, CI_callback)
        rospy.spin()
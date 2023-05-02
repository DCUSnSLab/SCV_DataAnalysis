#!/usr/bin/env python3
import rospy
import std_msgs.msg
import sensor_msgs.msg
from pympler.asizeof import asizeof
import datetime

pc2_pub = rospy.Publisher('/sampling_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
ci_pub = rospy.Publisher('/sampling_ci', sensor_msgs.msg.CompressedImage, queue_size=10)
f_flag = 0

def PC_callback(data):
    global f_flag

    # timestamp를 datetime 객체로 변환
    dt = datetime.datetime.fromtimestamp((data.header.stamp).to_sec())

    # datetime 객체에서 시간 정보 추출
    minute = dt.minute
    second = dt.second
    ms = dt.microsecond
    # print( sep=',')
    # timestamp = datetime.fromtimestamp(data.header.stamp)
    # # # 1s = 14 frame 여기는 10frame == 1초
    # 1 : 5/10  2,3 : 3/10   4, 5 : 2/10  6, 7 : 1/10  8, 9 : 1/10
    # 1초에 9번이라고 생각하면 됨
    if f_flag < 81:
        f_flag += 1
    else:
        pc2_pub.publish(data)
        print(minute, second, str(asizeof(data)), data.point_step, data.row_step, sep=',')
        f_flag = 0

def CI_callback(data):
    global f_flag

    if f_flag < 3:
        f_flag += 1
    else:
        ci_pub.publish(data)
        f_flag = 0

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node("Sampling_pc2")

        # Pointcloud2
        rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, PC_callback)

        # compressedImage
        # rospy.Subscriber('/zed2/zed_node/left/image_rect_color/compressed', sensor_msgs.msg.CompressedImage, CI_callback)
        rospy.spin()

# import rospy
# import std_msgs.msg
# import sensor_msgs.msg
# import time
#
# # rospy.Rate : 지정된 속도로 루프에서 잠을 잘 수 있는 편의 클래스
# # rospy.Timer : 지정된 속도로 콜백을 호출하기 위한 편의 클래스
#
# pc2_pub = rospy.Publisher('/sampling_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
# ci_pub = rospy.Publisher('/sampling_ci', sensor_msgs.msg.CompressedImage, queue_size=10)
#
# class Timer():
#     def __init__(self):
#         self.std = time.time()
#         self.sec = 0.1
#         self.hz = 1/self.sec
#     def check(self):
#         result = time.time() - self.std
#         if (result >= self.sec):
#             self.std = time.time()
#             return True
#         else:
#             return False
#
# timer = Timer()
#
# def PC_callback(data):
#     global timer
#     status = timer.check()
#     if(status):
#         pc2_pub.publish(data)
#         print(data)
#
# # def CI_callback(data):
# #     global timer
# #     status = timer.check()
# #     if (status):
# #         ci_pub.publish(data)
#
# if __name__ == '__main__':
#     while not rospy.is_shutdown():
#         rospy.init_node("Sampling_pc2")
#         # Pointcloud2
#         rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, PC_callback)
#         # compressedImage
#         # rospy.Subscriber('/zed2/zed_node/left/image_rect_color/compressed', sensor_msgs.msg.CompressedImage, CI_callback)
#         rospy.spin()
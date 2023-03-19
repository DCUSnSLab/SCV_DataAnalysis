#! /usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs import *
import numpy as np
from zed_interfaces.msg import ObjectsStamped

class Bbox_feature:
    def __init__(self):
        # Initialize ros publisher, ros subscriber
        # topic where we publish
        self.box_publisher = rospy.Publisher('bounding_boxes', Marker, queue_size=10)

        # subscribed Topic
        self.box_subscriber = rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, self.callback)

    # make 3d bounding box
    def draw_box(self, box_points):

        line_list = [Marker()] * 13
        for l in range(len(line_list)):
            line_list[l] = Marker()
            line_list[l].action = Marker().ADD
            line_list[l].header.frame_id = "map"
            line_list[l].ns = "line_" + str(line_list[l])
            line_list[l].pose.orientation.w = 1.0
            line_list[l].type = Marker().LINE_STRIP
            line_list[l].id = l
            line_list[l].scale.x = 0.1
            line_list[l].color.b = 1.0
            line_list[l].color.g = 1.0
            line_list[l].color.a = 1.0

        for i in range(1, 5):
            p = Point()
            d = Point()
            d.x = box_points[i - 1, 0]
            d.y = box_points[i - 1, 1]
            d.z = box_points[i - 1, 2]
            p.x = box_points[i, 0]
            p.y = box_points[i, 1]
            p.z = box_points[i, 2]
            line_list[i - 1].points.append(p)
            line_list[i - 1].points.append(d)
            box_publisher.publish(line_list[i - 1])

        for i in range(6, 10):
            p = Point()
            d = Point()
            d.x = box_points[i - 1, 0]
            d.y = box_points[i - 1, 1]
            d.z = box_points[i - 1, 2]
            p.x = box_points[i, 0]
            p.y = box_points[i, 1]
            p.z = box_points[i, 2]
            line_list[i - 2].points.append(d)
            line_list[i - 2].points.append(p)
            box_publisher.publish(line_list[i - 2])

        # 다른 포인트를 써서 해보자.

        for i in range(5):
            p = Point()
            d = Point()
            d.x = box_points[i, 0]
            d.y = box_points[i, 1]
            d.z = box_points[i, 2]
            p.x = box_points[i + 5, 0]
            p.y = box_points[i + 5, 1]
            p.z = box_points[i + 5, 2]
            # 8, 9, 10, 11
            line_list[i + 7].points.append(d)
            line_list[i + 7].points.append(p)
            box_publisher.publish(line_list[i + 7])



    def callback(self, data):
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
        self.draw_box(raw_point)

def main():
    Bbox_feature()
    rospy.init_node('bounding_box_publisher')
    rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        main()
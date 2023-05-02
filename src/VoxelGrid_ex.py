#! /usr/bin/env python3
import rospy
import std_msgs.msg
import sensor_msgs.msg
import time
import pcl

# rospy.Rate : 지정된 속도로 루프에서 잠을 잘 수 있는 편의 클래스
# rospy.Timer : 지정된 속도로 콜백을 호출하기 위한 편의 클래스

pc2_pub = rospy.Publisher('/sampling_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
f_flag = 0


def PC_callback(data):
    global f_flag

    def do_voxel_grid_downssampling(pcl_data, leaf_size):
        '''
        Create a VoxelGrid filter object for a input point cloud
        :param pcl_data: point cloud data subscriber
        :param leaf_size: voxel(or leaf) size
        :return: Voxel grid downsampling on point cloud
        :https://github.com/fouliex/RoboticPerception
        '''
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size)  # The bigger the leaf size the less information retained
        return vox.filter()

    #rate = rospy.Rate(0.5)  # ROS Rate at 0.5Hz
    if f_flag < 2:
        f_flag += 1
    else:
        cloud = (data)

        LEAF_SIZE = 0.01

        cloud = do_voxel_grid_downssampling(cloud, LEAF_SIZE)

        print(cloud)

        pc2_pub.publish(cloud)

        f_flag = 0
    #rate.sleep()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node("Voxel_pc2")
        # Pointcloud2
        rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, PC_callback)
        rospy.spin()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
class StaticMapPublisher:
    def __init__(self):
        rospy.init_node('static_map_publisher', anonymous=True)
        
        # 参数配置
        self.map_resolution = 0.1  
        self.map_width = 150        # 地图宽度 (15m / 0.05 = 300)
        self.map_height = 250       # 地图高度 (10m / 0.05 = 200)
        
        # 边框位置（相对于车体中心，单位：米）
        self.border_left = -2.6     # x = -3m
        self.border_right = 2.6     # x = 3m
        self.border_bottom = -2.0   # y = -2m
        
        # 边框线宽（米）
        self.border_thickness = 0.1  # 10cm线宽
        
        # 创建发布器
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        
        # 创建TF广播器（发布map到base_link的变换）
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 生成地图
        self.map_msg = self.create_map()
        
        rospy.loginfo("静态地图发布器已启动")
        rospy.loginfo("地图范围: x[-7.5, 7.5]m, y[-5, 5]m")
        rospy.loginfo("边框: x=-3m, x=3m, y=-2m")
    
    def create_map(self):
        """创建静态地图"""
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "odom"
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        
        # 设置地图原点：让车体(0,0)位于地图中心
        # 地图实际范围：x[-7.5, 7.5], y[-5, 5]
        map_msg.info.origin.position.x = - (self.map_width * self.map_resolution) / 2
        map_msg.info.origin.position.y = - (self.map_height * self.map_resolution) / 2
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.w = 1.0
        
        # 初始化地图数据（全0，自由空间）
        map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # 计算像素坐标转换
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # 创建坐标网格
        x_coords = np.arange(self.map_width) * self.map_resolution + origin_x
        y_coords = np.arange(self.map_height) * self.map_resolution + origin_y
        X, Y = np.meshgrid(x_coords, y_coords)
        
        # 绘制三条边框（代价=100，表示障碍物）
        half_thick = self.border_thickness / 2
        
        # 左边框: x = -3m
        left_mask = np.abs(X - self.border_left) < half_thick
        map_data[left_mask] = 100
        
        # 右边框: x = 3m
        right_mask = np.abs(X - self.border_right) < half_thick
        map_data[right_mask] = 100
        
        # 底边框: y = -2m
        bottom_mask = np.abs(Y - self.border_bottom) < half_thick
        map_data[bottom_mask] = 100
        
        # 展平为一维数组（ROS地图数据格式：行优先）
        map_msg.data = map_data.flatten().tolist()
        
        rospy.loginfo(f"地图生成完成: 障碍物格点数={np.sum(map_data > 0)}")
        
        return map_msg
    
    def run(self):
        rate = rospy.Rate(1)  # 10Hz
        
        while not rospy.is_shutdown():
            # 更新时间戳
            self.map_msg.header.stamp = rospy.Time.now()
            
            # 发布地图（latch=True确保新订阅者能收到）
            self.map_pub.publish(self.map_msg)
            
            # 发布TF变换
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = StaticMapPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
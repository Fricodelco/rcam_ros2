#!/usr/bin/python3
# -*- coding: utf-8 -*-
from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.parameter import Parameter
import cv2
from nav2_msgs.srv import ClearEntireCostmap


class CostmapAnalizer(Node):
    def __init__(self):
        super().__init__('costmap_analizer')
        simTime = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([simTime])
        self.subscription_1 = self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.map_callback,10)
        self.stop_distance = 1.0 #критическая дистанция до препятствия
        self.cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ClearEntireCostmap.Request()
        self.create_timer(1, self.clear_costmap_callback)
        
    def map_callback(self, map):
        blank_image = np.zeros((map.info.height, map.info.width), np.uint8)
        stop_robot = False
        stop_iter = blank_image.shape[1]/2 + self.stop_distance/map.info.resolution
        # начинаем движение по пикселям карты
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                # преобразование массива карты к двумерному виду
                k = j + (map.info.height - i - 1)*map.info.width
                blank_image[i][j] = 254 - map.data[int(k)]
                # если пиксель темный и ближе чем критическая дистанция до препятствия нужно остановится 
                if blank_image[i,j] <= 200 and stop_iter > j:
                    stop_robot=True
        if stop_robot is True:
            print("stop the robot")
        else:
            print("start_movement")
        """
            фрагмент ниже визуализирует полученный массив
        """
        # img = blank_image
        # cv2.imwrite('Test_gray.jpg', img)
        # scale_percent = 400
        # width = int(img.shape[1] * scale_percent / 100)
        # height = int(img.shape[0] * scale_percent / 100)
        # dim = (width, height)
        # resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        # cv2.imshow('img',img)
        # cv2.waitKey(1)

    """Очищаем костмапу раз в секунду для того чтобы на ней не оставалось старых препятствий 
    """
    def clear_costmap_callback(self):
        self.future = self.cli.call_async(self.req)

def main():
    rclpy.init()
    analizer = CostmapAnalizer()
    rclpy.spin(analizer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

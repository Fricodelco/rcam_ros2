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


class CostmapAnalyzer(Node):
    def __init__(self):
        super().__init__('costmap_analyzer')
        simTime = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([simTime])
        self.subscription_1 = self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.map_callback,10)
        self.stop_distance_meters = 1.0 #критическая дистанция до препятствия
        self.cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ClearEntireCostmap.Request()
        self.create_timer(1, self.clear_costmap_callback)
        self.hasLogged = False
        self.stop_iter = 0

    def is_obstacle_on_map_image(self, map_image):
        height = map_image.shape[0]
        width = map_image.shape[1]
        for i in range(0, width):
            for j in range(0, height):
                # если пиксель темный и ближе чем критическая дистанция до препятствия нужно остановится 
                if map_image[i,j] <= 200 and self.stop_iter > j:
                    self.get_logger().info('CostmapAnalyzer : obstacle at [{0} {1}]'.format(i,j))
                    map_image[i,j] = 0    #пометить точку-'препятствие'
                    return True
        return False


    def map_callback(self, map):
        stop_distance_px = self.stop_distance_meters/map.info.resolution
        self.stop_iter = map.info.width/2 + stop_distance_px
        if (self.hasLogged is False):
            self.get_logger().info('CostmapAnalyzer : stop thr {0} {1} px'.format(stop_distance_px, self.stop_iter))
            self.hasLogged = True

        map_image = np.zeros((map.info.height, map.info.width), np.uint8)
        stop_robot = False
        # начинаем движение по пикселям карты
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                # преобразование массива карты к двумерному виду
                k = j + (map.info.height - i - 1)*map.info.width
                map_image[i][j] = 254 - map.data[int(k)]

        if self.is_obstacle_on_map_image(map_image) is False:
            print("CostmapAnalyzer : no obstacle")
        
        """
            фрагмент ниже визуализирует полученный массив
        """
        img = map_image
        cv2.imwrite('/tmp/costmap_last.png', img)
        #scale_percent = 400
        #width = int(img.shape[1] * scale_percent / 100)
        #height = int(img.shape[0] * scale_percent / 100)
        #dim = (width, height)
        #resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        #cv2.imshow('img',img)
        #cv2.waitKey(1)

    """Очищаем костмапу раз в секунду для того чтобы на ней не оставалось старых препятствий 
    """
    def clear_costmap_callback(self):
        self.future = self.cli.call_async(self.req)

def main():
    rclpy.init()
    analizer = CostmapAnalyzer()
    rclpy.spin(analizer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

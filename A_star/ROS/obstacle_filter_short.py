#!/usr/bin/env python

import numpy as np
import rospy
import math
import sys
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from tricat_211.msg import FilteredObstacles, ObstaclePoint


class ObstacleFilter:
    def __init__(self):
        ## paraments init
        ob_config = rospy.get_param("Obstacle")
        self.obstacle_search_range = ob_config['obstacle_search_range']
        self.min_wall_length = ob_config['min_wall_length']

        ## lists
        self.ob_list = np.zeros((1, 3)) #(x from lidar, y from lidar, theta from lidar)

        ## ROS
        rospy.Subscriber('/obstacles', Obstacles, self.obstacle_callback)
        self.filtered_obstacles_pub = rospy.Publisher("/filtered_obstacles", FilteredObstacles, queue_size=10)

    def obstacle_callback(self, msg):
        self.ob_list = np.zeros((1, 3)) 

        ## circle filtering
        circle_list = msg.circles #.center.x/.center.y/.center.z/.radius
        for i in range(len(circle_list)): # check when (number of circles = 0)
            x = -circle_list[i].center.x #deleted (-) !!! check!!!!!
            y = circle_list[i].center.y
            d = math.sqrt(x ** 2 + y ** 2)
            theta = math.atan2(y,x) * math.pi / 180

            if d < self.obstacle_search_range:
                self.ob_list = np.append(self.ob_list, np.array([[x, y, theta]]), axis=0)
            else:
                continue

        ## wall filtering                
        segment_list = msg.segments #.first_point.x/.first_point.y/.first_point.z/last_point
        for i in range(len(segment_list)):
            start_x = segment_list[i].first_point.x
            start_y = segment_list[i].first_point.y
            end_x = segment_list[i].last_point.x
            end_y = segment_list[i].last_point.y
            distance = self.distance_boat_to_wall(start_x, start_y, end_x, end_y)
            length = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)

            if (distance < self.obstacle_search_range) and (abs(length) > self.min_wall_length):
                div_num = int(length/2 - 1)
                for i in range(div_num):
                    mid_x = start_x + (end_x-start_x)*(i/div_num)
                    mid_y = start_y + (end_y-start_y)*(i/div_num)
                    w_theta = math.atan2(mid_y,mid_x) * math.pi / 180
                    self.ob_list = np.append(self.ob_list, np.array([[mid_x, mid_y, w_theta]]), axis = 0)
                self.ob_list = np.append(self.ob_list, np.array([[end_x, end_y, (math.atan2(end_y, end_x) * math.pi / 180)]]), axis = 0)
            else:
                continue
        
        self.ob_list = np.delete(self.ob_list, 0, axis=0)

    def distance_boat_to_wall(self, start_x, start_y, end_x, end_y):
        boat_to_start = start_x**2 + start_y**2 #square value
        boat_to_end = end_x**2 + end_y**2 # square value
        start_to_end = (start_x-end_x)**2 + (start_y-end_y)**2 #square value
        cosin_value = (boat_to_start + start_to_end - boat_to_end)/(2*math.sqrt(boat_to_start)*math.sqrt(start_to_end))
        angle_value = math.acos(cosin_value)
        boat_to_line = abs(math.sqrt(boat_to_start)*math.sin(angle_value))
        return boat_to_line

    def filtered_obstacles_publisher(self):
        filtered_obstacles = FilteredObstacles()
        for i in range(len(self.ob_list)):
            filtered_obstacles.obstacle.append(ObstaclePoint(x=self.ob_list[i][0], y=self.ob_list[i][1], theta = self.ob_list[i][2]))
        self.filtered_obstacles_pub.publish(filtered_obstacles)


def main():
    rospy.init_node('Obstacle_Filter', anonymous=False)
    f = ObstacleFilter()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        f.filtered_obstacles_publisher()

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()


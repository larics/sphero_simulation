#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import reynolds_rules_functions as rr
import os
import sys
import math
import yaml
import roslaunch
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Bool

class ReynoldsController:
    
    def __init__(self):
        
        
        self.num_of_robots = 10
        
        self.vel_max = 2
        self.coh_strength = 5
        self.sep_strength = 1.5
        self.alig_strength = 0.5
        self.nav_strength = 0.5
        self.avoid_strength = 10
        self.avoid_distance = 1
        
        self.nav_point = [0 , 5]
        
        
        self.fov_max_dist = 1.5
        
        
        
        self.velocities_subscribed = [Twist().linear] * self.num_of_robots
        self.positions_subscribed =  [Pose().position] * self.num_of_robots
        self.rotations_subscribed =  [Pose().orientation] * self.num_of_robots
        self.robot_subsribers = self.create_subscribers_odom()
        self.robot_publishers = self.create_publishers_cmd_vel()
        self.occupancy_grid_coordinates = []
        self.map_subscriber = self.create_subsriber_map()
        
        
        self.RulesLib = rr.ReynoldsRules(self.num_of_robots, self.alig_strength, self.coh_strength, self.sep_strength, self.nav_strength, self.nav_point, self.avoid_strength, self.avoid_distance, fov_max_dist = self.fov_max_dist, fov_max_angle = 2 * np.pi) #Class with raynolds rules functions
    
    def map_to_coordinates(self, occupancy_grid_msg):
        # Extract map information
        resolution = occupancy_grid_msg.info.resolution
        origin = occupancy_grid_msg.info.origin
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height

        # Calculate coordinates
        coordinates = []
        for y in range(height):
            for x in range(width):
                index = y * width + x
                occupancy_value = occupancy_grid_msg.data[index]

                # Check if the cell is occupied (you can adjust this condition based on your needs)
                if occupancy_value > 50:
                    # Calculate real-world coordinates
                    real_x = origin.position.x + (x + 0.5) * resolution
                    real_y = origin.position.y + (y + 0.5) * resolution

                    coordinates.append((real_x, real_y))

        return coordinates    
        
    def map_callback(self, data):
        self.occupancy_grid_coordinates = self.map_to_coordinates(data)
    
    def create_subsriber_map(self):
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
    
    def create_subscribers_odom(self):
        for i in range(self.num_of_robots):
            rospy.Subscriber("/robot_{}/odom".format(i), Odometry, self.generate_callback(i))
        return

    def generate_callback(self, subscriber_number):
        def callback(data):
            self.velocities_subscribed[subscriber_number] = data.twist.twist.linear
            self.positions_subscribed[subscriber_number] = data.pose.pose.position
            self.rotations_subscribed[subscriber_number] = data.pose.pose.orientation
        return callback


    
    def create_publishers_cmd_vel(self):
        
        publishers = {}
        for i in range(self.num_of_robots):
            publishers[f'pub_{i}'] = rospy.Publisher(f'/robot_{i}/cmd_vel', Twist, queue_size = 1)
        return publishers

        
    def limit_vel(self, vel : Twist()):
        for i in range(self.num_of_robots):
            norm = (vel[i].linear.x **2 + vel[i].linear.y ** 2) ** 0.5
            if norm >= self.vel_max:
                vel[i].linear.x = vel[i].linear.x / norm * self.vel_max
                vel[i].linear.y = vel[i].linear.y / norm * self.vel_max
        return vel

    def run(self):
        while not rospy.is_shutdown():

            vel = [Twist()] * self.num_of_robots

            
            vel_coh = self.RulesLib.cohesion(self.positions_subscribed)
            vel_sep = self.RulesLib.separation(self.positions_subscribed)
            vel_alig = self.RulesLib.alignment(self.positions_subscribed, self.rotations_subscribed, self.velocities_subscribed)
            vel_nav = self.RulesLib.navigation(self.positions_subscribed)
            vel_avoid = self.RulesLib.avoidance(self.positions_subscribed, self.occupancy_grid_coordinates)
            
            for i in range(self.num_of_robots):
                vel[i].linear.x = (vel_coh[i].linear.x - vel_sep[i].linear.x + vel_alig[i].linear.x + vel_nav[i].linear.x - vel_avoid[i].linear.x)
                vel[i].linear.y = (vel_coh[i].linear.y - vel_sep[i].linear.y + vel_alig[i].linear.y + vel_nav[i].linear.y - vel_avoid[i].linear.y)
                
                vel_limited = self.limit_vel(vel)

                self.robot_publishers[f'pub_{i}'].publish(vel_limited[i])


if __name__ == '__main__':
    
    try:
        rospy.init_node('ReynoldsController', anonymous=False)
        #rate = rospy.Rate(10)
        controller = ReynoldsController()
        controller.run()
        #rate.sleep()
          
    except rospy.ROSInterruptException:
        pass
    
    

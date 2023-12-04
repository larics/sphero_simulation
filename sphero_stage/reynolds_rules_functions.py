from __future__ import print_function, division
import our_code as oc
import os
import sys
import math
import yaml
import roslaunch
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Bool

class ReynoldsRules:
    def __init__ (self, num_of_robots, align_strength, coh_strength, sep_strength, nav_strength, nav_point, avoid_strength, avoid_distance, fov_max_dist = 1, fov_max_angle = 2 * np.pi):
        self.num_of_robots = num_of_robots
        self.fov_max_dist = fov_max_dist
        self.fov_max_angle = fov_max_angle
        self.align_strength = align_strength
        self.coh_strength = coh_strength
        self.sep_strength = sep_strength
        self.nav_strength = nav_strength
        self.nav_point = nav_point
        self.avoid_strength = avoid_strength
        self.avoid_distance = avoid_distance
        
        
    def distance(self, x1, y1, x2, y2): return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def check_if_neighbor(self, current_robot_position, other_robot_position):
        """
        Method that searches for neighbor robots
        :param current_robot_position: TIP, current robot position
        :param other_robot_position: TIP, position of other robot in the world
        return: bool, true - if robot is in the FOV of the current robot, false - otherwise
        """

        [x1, y1] = current_robot_position
        [x2, y2] = other_robot_position

        # Compute distance and angle between current and other robot and compare them to FOV parameters
        dist = self.distance(x1, y1, x2, y2)
        angle = math.atan2((y2 - y1), (x2 - x1))
        return True if (dist < self.fov_max_dist and abs(angle) < self.fov_max_angle) else False

    def create_cohesion_force(self, current_robot_position, local_center_position):
        """
        Method that creates cohesion force to attract robots to form a group
        :param current_robot_position: TIP, position of current robot
        :param neighbor_robot_position: TIP, local center position
        return: list, cohesion force vector
        """

        # Create cohesion force by substracting position vectors
        force = local_center_position - current_robot_position

        return force

    def create_separation_force(self, current_robot_position, neighbor_robot_position):
        """
        Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
        :param current_robot_position: TIP, position of current robot
        :param neighbor_robot_position: TIP, position of other robot in the world
        return: list, weighted force vector
        """
        #print(current_robot_position, neighbor_robot_position)
        [x1, y1] = current_robot_position
        [x2, y2] = neighbor_robot_position

        # Compute distance current and other robot
        dist1 = (((x2 - x1) ** 2) + ((y2 - y1)) ** 2) ** 0.5
        
        # Create weighted force by substracting position vectors
        force = neighbor_robot_position - current_robot_position 
        #force = force/np.linalg.norm(force) / (distance ** 2)
        # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])
        force = force/(dist1 ** 2)
        return force

    def create_alignment_force(self, current_robot_velocity, local_center_velocity):
        """
        Method that creates alignment force to attract robots to form a group
        :param current_robot_velocity: TIP, position of current robot
        :param neighbor_robot_velocity: TIP, local center position
        return: list, alignment force vector
        """

        # Create alignment force by substracting position vectors
        force = local_center_velocity - current_robot_velocity

        return force

    def create_navigation_force(self, current_robot_position, goal_position):
        """
        Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
        :param current_robot_position: TIP, position of current robot
        :param neighbor_robot_position: TIP, position of other robot in the world
        return: list, weighted force vector
        """
        [x1, y1] = current_robot_position
        [x2, y2] = goal_position[0], goal_position[1]

        # Compute distance current and other robot
        # distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

        # Create weighted force by substracting position vectors
        force = [x2, y2] - current_robot_position

        return force


    def cohesion(self, positions):
        cohesion_forces = []

        for i in range(self.num_of_robots):
            neighbor_count = 0
            current_robot_position = np.array([positions[i].x, positions[i].y])
            local_center_position = np.array((1,2), dtype='float64')

            for j in range(self.num_of_robots):
                other_robot_position = np.array([positions[j].x, positions[j].y])
                if i == j or (current_robot_position == other_robot_position).all():
                    continue

            
                if self.check_if_neighbor(current_robot_position, other_robot_position):
                    local_center_position += other_robot_position
                    neighbor_count += 1
            if neighbor_count:
                local_center_position = local_center_position / neighbor_count
                current_robot_cohesion_force = self.create_cohesion_force(current_robot_position, local_center_position)
    
            else: current_robot_cohesion_force = [0, 0]
            
            cohesion_forces.append(current_robot_cohesion_force)
        vel = []
        for i in range(self.num_of_robots):
            vel.append(Twist())
       
        for i in range(self.num_of_robots):
            vel[i].linear.x = self.coh_strength * (cohesion_forces[i][0])
            vel[i].linear.y = self.coh_strength * (cohesion_forces[i][1])
        #print(vel)
        return vel

    def separation(self, positions):
        separation_forces = []
        for i in range(self.num_of_robots):
            neighbor_count = 0
            current_robot_position = np.array([positions[i].x, positions[i].y])
            total_separation_force = np.zeros(2)

            for j in range(self.num_of_robots):
                other_robot_position = np.array([positions[j].x, positions[j].y])
                if i == j or (current_robot_position == other_robot_position).all():
                    continue

                
                if self.check_if_neighbor(current_robot_position, other_robot_position):
                    neighbor_count += 1
                    separation_force = self.create_separation_force(current_robot_position, other_robot_position)
                    total_separation_force += separation_force
                
            if neighbor_count:
                total_separation_force = total_separation_force / neighbor_count            
            else: total_separation_force = [0, 0]

            separation_forces.append(total_separation_force)

        vel = []
        for i in range(self.num_of_robots):
            vel.append(Twist())
        for i in range(self.num_of_robots):
            vel[i].linear.x = round(self.sep_strength * separation_forces[i][0], 2)
            vel[i].linear.y = round(self.sep_strength * separation_forces[i][1], 2)

        return vel

    def alignment(self, positions, rotations, velocities):
        alignment_forces = []

        for i in range(self.num_of_robots):
            current_robot_velocity = np.array([round(velocities[i].x, 2), round(velocities[i].y, 2)])
            current_robot_position = np.array([positions[i].x, positions[i].y])

            local_center_velocity = 0
            neighbor_count = 0
            for j in range(self.num_of_robots):
                other_robot_velocity = np.array([round(velocities[j].x, 2), round(velocities[j].y, 2)])
                other_robot_position = np.array([positions[j].x, positions[j].y])

                #if velocities[j].y < 0.01:
                #    other_robot_velocity = np.dot(rot_matrix(other_robot_rotation), other_robot_velocity)

                if i == j or (other_robot_position == current_robot_position).all():
                    continue

                if self.check_if_neighbor(current_robot_position, other_robot_position):
                    neighbor_count += 1
                    local_center_velocity += other_robot_velocity

            if neighbor_count:
                local_center_velocity = local_center_velocity / neighbor_count
                current_robot_alignment_force = self.create_alignment_force(current_robot_velocity, local_center_velocity)            
            else: current_robot_alignment_force = [0, 0]

            
            
            alignment_forces.append(current_robot_alignment_force)

        vel = []
        for i in range(self.num_of_robots):
            vel.append(Twist())

        for i in range(self.num_of_robots):
            vel[i].linear.x = self.align_strength * (alignment_forces[i][0])
            vel[i].linear.y = self.align_strength * (alignment_forces[i][1])

        return vel
    
    def navigation(self, positions):
        navigation_forces = []

        for i in range(self.num_of_robots):
            current_robot_position = np.array([positions[i].x, positions[i].y])
            
            current_robot_navigation_force = self.create_navigation_force(current_robot_position, self.nav_point)
            
            navigation_forces.append(current_robot_navigation_force)
        
        vel = []
        for i in range(self.num_of_robots):
            vel.append(Twist())
       
        for i in range(self.num_of_robots):
            vel[i].linear.x = self.nav_strength * (navigation_forces[i][0])
            vel[i].linear.y = self.nav_strength * (navigation_forces[i][1])
        return vel
    
    def avoidance(self, positions, occupancy_grid_coordinates):
        avoidance_forces = []

        for i in range(self.num_of_robots):
            current_robot_position = np.array([positions[i].x, positions[i].y])
            
            closest_object_coordinates, distance_from_closest_object = self.find_closest_object_point_avoidance(current_robot_position, occupancy_grid_coordinates)
            print(closest_object_coordinates, distance_from_closest_object)
            if distance_from_closest_object < self.avoid_distance:
                current_robot_navigation_force = self.create_avoidance_force(current_robot_position, closest_object_coordinates, distance_from_closest_object)
            
                avoidance_forces.append(current_robot_navigation_force)
            else: avoidance_forces.append([0, 0])
        
        vel = []
        for i in range(self.num_of_robots):
            vel.append(Twist())
       
        for i in range(self.num_of_robots):
            vel[i].linear.x = self.avoid_strength * (avoidance_forces[i][0])
            vel[i].linear.y = self.avoid_strength * (avoidance_forces[i][1])
        return vel
    
    def find_closest_object_point_avoidance(self, current_robot_position, occupancy_grid_coordinates):
        [x1, y1] = current_robot_position
        min_dist = 100
        min_point_dist = 1
        for i in occupancy_grid_coordinates:
            print(i)
            [x2, y2] = i
            dist = self.distance(x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                min_point_dist = i
        
        return (min_dist, min_point_dist)
    
    def create_avoidance_force(self, current_robot_position, closest_object_coordinates, distance_from_closest_object):
        
        force = closest_object_coordinates - current_robot_position
        
        return force * self.avoid_distance / distance_from_closest_object
    
    
    
    def create_obstacle_force(self, current_robot_position, obstacle):
        """
        Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
        :param current_robot_position: TIP, position of current robot
        :param neighbor_robot_position: TIP, position of other robot in the world
        return: list, weighted force vector
        """
        # rospy.logwarn(goal_position)
        # rospy.logerr(current_robot_position)
        [x1, y1] = current_robot_position
        [x2, y2] = obstacle

        # Compute distance current and other robot
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        # print(distance)

        # Create weighted force by substracting position vectors
        force = current_robot_position - obstacle
        # force = force / np.linalg.norm(force) / (distance ** 2)
        force = force / np.linalg.norm(force) / (distance ** 2)
        # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])

        return force
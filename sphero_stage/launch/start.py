#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import os
import sys
import math
import yaml
import roslaunch
import rospy
import rospkg

def distribute_circle(k, n, center_x=0, center_y=0, radius=1):
    x = radius * math.cos(k / n * 2 * math.pi) + center_x
    y = radius * math.sin(k / n * 2 * math.pi) + center_y
    return x, y

def distribute_line(k, n, center_x=0, center_y=0, separation=0.5, direction='horizontal'):
    if direction == 'horizontal':
        x = center_x + (k - (n - 1) // 2) * separation
        y = center_y
    else:
        x = center_x
        y = center_y + (k - (n - 1) // 2) * separation
    return x, y

def create_files(resources, config):
    map_name = config['map_name']
    num_of_robots = config['num_of_robots']
    distribution = config['distribution']
    params = config['distribution_list'][distribution]

    template_file = "{}/world_templates/{}.temp".format(resources, map_name)
    new_map_file  = "{}/worlds/{}_{}.world".format(resources, map_name, num_of_robots)

    if not os.path.isfile(template_file):
        print("\033[31mUnable to create files - missing template! \033[0m")
        return False

    proto_line = "sphero( pose [ {x:.3f} {y:.3f} 0 0.000 ] name \"sphero_{k}\" color \"{color}\")\n"

    with open(new_map_file, 'w') as output, open(template_file, 'r') as temp:
        output.write(temp.read())

        for k in range(num_of_robots):
            if distribution == 'circle':
                x, y = distribute_circle(k, num_of_robots, **params)
            elif distribution == 'line':
                x, y = distribute_line(k, num_of_robots, **params)
            args = {'x': x, 'y': y, 'k': k, 'color': 'blue'}
            output.write(proto_line.format(**args))

    return True


def main():
    start_rviz = 'true' if '--rviz' in sys.argv else 'false'

    # Set up launch variables. These are hard-coded and they shouldn't be changed.
    package = rospkg.RosPack().get_path('sphero_stage')
    resources = package + '/resources'
    with open(package + '/launch/launch_params.yaml', 'r') as stream:
        config = yaml.full_load(stream)
    num_of_robots = config['num_of_robots']
    map_name = config['map_name']
    cli_args = [package + '/launch/setup_sim.xml',
                'num_of_robots:=' + str(num_of_robots),
                'map_name:=' + map_name,
                'start_rviz:=' + start_rviz]

    # Create a world file from template using the same name and specified number of robots.
    print("\033[36m \nCreating world file with {} robots and map '{}'.\033[0m".format(num_of_robots, map_name))
    if not create_files(resources, config):
        return

    print("\033[36mCalling the setup launch file. \033[0m")
    # Prepare for launching.
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    # Launch!
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

    print("\033[92mStage simulator launched. Press Ctrl-C to exit when done. \033[0m")

    # Keep it from exiting.
    try:
        launch.spin()
    finally:
        # After Ctrl+C, stop all nodes from running.
        launch.shutdown()

if __name__ == '__main__':
    rospy.init_node('stage_launcher', anonymous=True)

    print("\033[36m \nInitialized launcher node. Starting launch process. \033[0m")

    main()










#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import random
import numpy as np
pi = np.pi


def generate_random_color_list(possible_colors, n):
    result = []
    color_count = {color: 0 for color in possible_colors}
    
    while len(result) < n:
        min_count = min(color_count.values())
        available_colors = [color for color in possible_colors if color_count[color] == min_count]
        color = random.choice(available_colors)
        result.append(color)
        color_count[color] += 1
        
    return result




class Cube:
    def __init__(self, output_directory, cube_side, mass, r_range, theta_range, r_fixed, theta_fixed, 
                 yaw_range, yaw_fixed, color):
        self.output_directory = output_directory
        self.r_range = r_range
        self.theta_range = theta_range
        self.r_fixed = r_fixed
        self.theta_fixed = theta_fixed
        self.yaw_range = yaw_range
        self.yaw_fixed = yaw_fixed
        self.color = color
        self.cube_side = cube_side
        self.z_pos = self.cube_side / 2
        self.mass = mass
        self.position = self.generate_position()

        
    def generate_position(self):
        if self.r_fixed is not None:
            r_pos = self.r_fixed 
        else:
            r_pos = random.uniform(*self.r_range)
       
        if self.theta_fixed is not None:
            theta_pos = self.theta_fixed 
        else:    
            theta_pos = random.uniform(*self.theta_range)
        
        if self.yaw_fixed is not None:
            yaw = self.yaw_fixed
        else:
            yaw = random.uniform(*self.yaw_range)
            
        x_pos = r_pos * np.cos(theta_pos)
        y_pos = r_pos * np.sin(theta_pos)
            
        return [x_pos, y_pos, self.z_pos, yaw]

    
    def generate_urdf(self, cube_index):
        x_pos, y_pos, z_pos, yaw = self.position

        colors = {
            'Red': '1 0 0 1',
            'Blue': '0 0 1 1',
            'Green': '0 1 0 1',
            'Yellow': '1 1 0 1',
        }

        color_rgba = colors.get(self.color, colors['Red'])

        urdf_content = ""
        urdf_content += "<?xml version='1.0'?>\n"
        urdf_content += "<robot name='cube_{}'>\n".format(cube_index)
        urdf_content += "\t<link name='world'/>\n"
        urdf_content += "\t<joint name='world_joint' type='floating'>\n"
        urdf_content += "\t\t<parent link='world'/>\n"
        urdf_content += "\t\t<child link='cube'/>\n"
        urdf_content += "\t\t<origin xyz='0 0 0' rpy='0 0 0'/>\n"
        urdf_content += "\t</joint>\n"
        urdf_content += "\t<link name='cube'>\n"
        urdf_content += "\t\t<visual>\n"
        urdf_content += "\t\t\t<origin xyz='{} {} {}' rpy='0 0 {}'/>\n".format(x_pos, y_pos, z_pos, yaw)
        urdf_content += "\t\t\t<geometry>\n"
        urdf_content += "\t\t\t\t<box size='{} {} {}' />\n".format(self.cube_side, self.cube_side, self.cube_side)
        urdf_content += "\t\t\t</geometry>\n"
        urdf_content += "\t\t\t<material name='{}'>\n".format(self.color)
        urdf_content += "\t\t\t\t<color rgba='{}' />\n".format(color_rgba)
        urdf_content += "\t\t\t</material>\n"
        urdf_content += "\t\t</visual>\n"
        urdf_content += "\t\t<collision>\n"
        urdf_content += "\t\t\t<origin xyz='{} {} {}' rpy='0 0 {}'/>\n".format(x_pos, y_pos, z_pos, yaw)
        urdf_content += "\t\t\t<geometry>\n"
        urdf_content += "\t\t\t\t<box size='{} {} {}' />\n".format(self.cube_side, self.cube_side, self.cube_side)
        urdf_content += "\t\t\t</geometry>\n"
        urdf_content += "\t\t</collision>\n"
        urdf_content += "\t\t<inertial>\n"
        urdf_content += "\t\t\t<origin xyz='{} {} {}' rpy='0 0 {}'/>\n".format(x_pos, y_pos, z_pos, yaw)
        urdf_content += "\t\t\t<mass value='{}' />\n".format(self.mass)
        urdf_content += "\t\t\t<inertia \n"
        urdf_content += "\t\t\t\tixx='{}' \n".format(self.mass * self.cube_side**2 / 6)
        urdf_content += "\t\t\t\tixy='0.0' \n"
        urdf_content += "\t\t\t\tixz='0.0' \n"
        urdf_content += "\t\t\t\tiyy='{}' \n".format(self.mass * self.cube_side**2 / 6)
        urdf_content += "\t\t\t\tiyz='0.0' \n"
        urdf_content += "\t\t\t\tizz='{}' />\n".format(self.mass * self.cube_side**2 / 6)
        urdf_content += "\t\t</inertial>\n"
        urdf_content += "\t</link>\n"
        urdf_content += "\t<gazebo reference='cube'>\n"
        urdf_content += "\t\t<material>Gazebo/{}</material>\n".format(self.color)
        urdf_content += "\t</gazebo>\n"
        urdf_content += "</robot>"

        os.makedirs(self.output_directory, exist_ok=True)
        urdf_file_path = os.path.join(self.output_directory, 'cube_{}.urdf'.format(cube_index))

        with open(urdf_file_path, 'w') as f:
            f.write(urdf_content)

        print(f"URDF file created successfully at {urdf_file_path}")    
            
            
            
    
class CubeGenerator:
    def __init__(self, output_directory, launch_directory,moveit_launch_directory, num_cubes, cube_side, mass,
                 r_range, theta_range, r_fixed, theta_fixed, yaw_range, yaw_fixed, color):
        
        self.output_directory = output_directory
        self.launch_directory = launch_directory
        self.moveit_launch_directory = moveit_launch_directory
        self.num_cubes = num_cubes
        self.r_range = r_range
        self.theta_range = theta_range
        self.r_fixed = r_fixed
        self.theta_fixed = theta_fixed
        self.yaw_range = yaw_range
        self.yaw_fixed = yaw_fixed
        self.color = color
        self.cubes = []
        self.cube_side = cube_side
        self.min_separation = self.cube_side * np.sqrt(2)
        self.mass = mass
        

        
    def generate_cubes(self):
        for i in range(self.num_cubes):
            while True:
                cube = Cube(self.output_directory, 
                            self.cube_side,
                            self.mass,
                            self.r_range[i], 
                            self.theta_range[i], 
                            self.r_fixed[i], 
                            self.theta_fixed[i], 
                            self.yaw_range[i], 
                            self.yaw_fixed[i],
                            self.color[i])
                if self.is_valid_position(cube):
                    self.cubes.append(cube)
                    break

    def is_valid_position(self, new_cube):
        for existing_cube in self.cubes:
            dist_x = new_cube.position[0] - existing_cube.position[0]
            dist_y = new_cube.position[1] - existing_cube.position[1]
            distance = np.sqrt(dist_x**2 + dist_y**2)
            if distance < self.min_separation:
                return False
        return True

    def generate_urdf_files(self):
        for i, cube in enumerate(self.cubes):
            cube.generate_urdf(i+1)
  

    def generate_launch_files(self):
        for i in range(self.num_cubes):
            launch_content = ""
            launch_content += "<?xml version='1.0'?>\n"
            launch_content += "<launch>\n\n"
            launch_content += "\t<!-- Load the cube -->\n"
            launch_content += "\t<param name='cube_description' command=\"$(find xacro)/xacro --inorder '$(find dobot_description)/urdf/cube_{}.urdf'\" />\n".format(i+1)
            launch_content += "\t<param name='use_gui' value='true' />\n\n"
            launch_content += "\t<!-- Spawn the cube in Gazebo -->\n"
            launch_content += "\t<node name='spawn_model' pkg='gazebo_ros' type='spawn_model' respawn='false' output='screen' \n"
            launch_content += "\t\t args='-urdf -model cube_{} -param cube_description' />\n\n".format(i+1)
            launch_content += "</launch>"

            os.makedirs(self.launch_directory, exist_ok=True)
            launch_file_path = os.path.join(self.launch_directory, 'spawn_cube_{}.launch'.format(i+1))

            with open(launch_file_path, 'w') as f:
                f.write(launch_content)
                
            print(f"Launch file created successfully at {launch_file_path}")    
                

    def generate_world_file(self):
        world_content = ""
        world_content += "<?xml version='1.0'?>\n"
        world_content += "<launch>\n\n"
        world_content += "\t<!-- Spawn empty world -->\n"
        world_content += "\t<include file='$(find gazebo_ros)/launch/empty_world.launch'>\n"
        world_content += "\t\t<arg name='paused' value='true'/>\n"
        world_content += "\t</include>\n\n"
        world_content += "\t<group ns='dobot'>\n"
        world_content += "\t\t<include file='$(find dobot_description)/launch/world_dobot.launch' />\n"
        world_content += "\t</group>\n\n"
        
        for i in range(self.num_cubes):
            world_content += "\t<group ns='cube_{}'>\n".format(i+1)
            world_content += "\t\t<include file='$(find dobot_description)/launch/spawn_cube_{}.launch' />\n".format(i+1)
            world_content += "\t</group>\n\n"
        
        world_content += "</launch>"

        os.makedirs(self.launch_directory, exist_ok=True)
        world_file_path = os.path.join(self.launch_directory, 'world.launch')

        with open(world_file_path, 'w') as f:
            f.write(world_content)
            
        print(f"Main launch file created successfully at {world_file_path}")
        
        
    def generate_moveit_launch_file(self):
        moveit_content = ""
        moveit_content += "<?xml version='1.0'?>\n"
        moveit_content += "<launch>\n\n"
        moveit_content += "\t<!-- Spawn empty world -->\n"
        moveit_content += "\t<include file='$(find gazebo_ros)/launch/empty_world.launch'>\n"
        moveit_content += "\t\t<arg name='paused' value='true'/>\n"
        moveit_content += "\t</include>\n\n"
        moveit_content += "\t<group ns='dobot'>\n"
        moveit_content += "\t\t<include file='$(find dobot_description)/launch/world_dobot.launch' />\n"
        moveit_content += "\t\t<include file = '$(find moveit_dobot)/launch/move_group.launch' />\n"
        moveit_content += "\t\t<arg name='use_rviz' default='true' />\n"
        moveit_content += "\t\t<include file='$(find moveit_dobot)/launch/moveit_rviz.launch' if='$(arg use_rviz)'>\n"
        moveit_content += "\t\t\t<arg name='rviz_config' value='$(find moveit_dobot)/launch/moveit.rviz'/>\n"
        moveit_content += "\t\t</include>\n"
        moveit_content += "\t</group>\n\n"
        
        for i in range(self.num_cubes):
            moveit_content += "\t<group ns='cube_{}'>\n".format(i+1)
            moveit_content += "\t\t<include file='$(find dobot_description)/launch/spawn_cube_{}.launch' />\n".format(i+1)
            moveit_content += "\t</group>\n\n"
        
        moveit_content += "</launch>"

        os.makedirs(self.moveit_launch_directory, exist_ok=True)
        moveit_file_path = os.path.join(self.moveit_launch_directory, 'dobot_cubes_sim.launch')

        with open(moveit_file_path, 'w') as f:
            f.write(moveit_content)
            
        print(f"Moveit simulation created succesfully at {moveit_file_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate cubes for Gazebo simulation.')
    parser.add_argument('--num_cubes', type=int, default=1, help='Number of cubes to generate.')
    parser.add_argument('--cube_side', type=float, default=0.03, help='Cube side length')
    parser.add_argument('--mass', type=float, default=0.03, help='Cube mass')
    parser.add_argument('--r_range', type=float, nargs=2, default=[0.05, 0.35], help='Range for radial position (min max).')
    parser.add_argument('--theta_range', type=float, nargs=2, default=[-90, 90], help='Range for azimutal position (min max) (in degrees).')
    parser.add_argument('--r_fixed', type=float, nargs='+', default=None, help='Fixed r positions for each cube.')
    parser.add_argument('--theta_fixed', type=float, nargs='+', default=None, help='Fixed theta positions for each cube (in degrees)')
    parser.add_argument('--yaw_range', type=float, nargs=2, default=[-90, 90], help='Range for yaw (min max) (in degrees)')
    parser.add_argument('--yaw_fixed', type=float, nargs='+', default=None, help='Fixed yaw values for each cube (in degrees)')
    parser.add_argument('--color', type=str, nargs='+', default=None, help='Colors for each cube.')
    
    
    
    urdf_directory = '../urdf'
    launch_directory = '../launch'
    moveit_launch_directory = '../../moveit_dobot/launch'
    
    
    args = parser.parse_args()
    
    if args.r_fixed:
        if len(args.r_fixed) != args.num_cubes:
            raise ValueError("Length of r_fixed must be equal to num_cubes")
        args.r_range = [None] * args.num_cubes
    else:
        args.r_fixed = [None] * args.num_cubes
        if args.num_cubes != 1:
            args.r_range = [args.r_range] * args.num_cubes
        
    if args.theta_fixed:
        if len(args.theta_fixed) != args.num_cubes:
            raise ValueError("Length of theta_fixed must be equal to num_cubes")
        args.theta_fixed = np.radians(args.theta_fixed)
        args.theta_range = [None] * args.num_cubes
    else:
        args.theta_fixed = [None] * args.num_cubes
        args.theta_range = np.radians(args.theta_range)
        if args.num_cubes != 1:
            args.theta_range = [args.theta_range] * args.num_cubes
        
    if args.yaw_fixed:
        if len(args.yaw_fixed) != args.num_cubes:
            raise ValueError("Length of yaw_fixed must be equal to num_cubes")
        args.yaw_fixed = np.radians(args.yaw_fixed)
        args.yaw_range = [None] * args.num_cubes
    else:
        args.yaw_fixed = [None] * args.num_cubes
        args.yaw_range = np.radians(args.yaw_range)
        if args.num_cubes != 1:
            args.yaw_range = [args.yaw_range] * args.num_cubes
        
    if args.color:
        if len(args.color) != args.num_cubes:
            raise ValueError("Length of colors must be equal to num_cubes")
    else:
        possible_colors = ['Red', 'Blue', 'Green', 'Yellow']
        args.color = generate_random_color_list(possible_colors, args.num_cubes)
            
            

    manager = CubeGenerator(
        output_directory = urdf_directory,
        launch_directory = launch_directory, 
        moveit_launch_directory = moveit_launch_directory,
        num_cubes = args.num_cubes,
        cube_side = args.cube_side,
        mass = args.mass,
        r_range = args.r_range,
        theta_range = args.theta_range,
        r_fixed = args.r_fixed,
        theta_fixed = args.theta_fixed,
        yaw_range = args.yaw_range,
        yaw_fixed = args.yaw_fixed,
        color = args.color
    )
    
    manager.generate_cubes()
    manager.generate_urdf_files()
    manager.generate_launch_files()
    manager.generate_world_file()
    manager.generate_moveit_launch_file()
    



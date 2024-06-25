#!/usr/bin/env python


import rospy
import xml.etree.ElementTree as ET
import sys
import moveit_commander
import geometry_msgs.msg
import argparse
import os
import numpy as np
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion



class URDFParser:
    def __init__(self, launch_file):
        rospy.init_node('urdf_parser', anonymous=True)
        self.launch_file = launch_file
        self.root = self._parse_launch_file()
        self.num_cubes = self.count_cube_groups()

    def _parse_launch_file(self):
        tree = ET.parse(self.launch_file)
        return tree.getroot()

    def count_cube_groups(self):
        count = 0
        for group in self.root.findall('.//group'):
            group_name = group.attrib.get('ns', '')
            if group_name.startswith('cube_'):
                count += 1
                
        return count
        
        
    def collect_cube_data(self):
        positions = []
        orientations = []
        colors = []
        cube_sizes = []
        
        for group in self.root.findall('.//group'):
            group_name = group.attrib.get('ns', '')
            if group_name.startswith('cube_'):
                cube = CubeData(group_name, 'cube')
                if cube.position is not None and cube.orientation is not None:
                    positions.append(cube.position)
                    orientations.append(cube.orientation)
                    colors.append(cube.color)
                    cube_sizes.append(cube.size)
                else:
                    rospy.logwarn(f"No se pudo obtener la posición y orientación del enlace 'cube' en el grupo '{group_name}'.")
        
        self.positions = positions
        self.orientations = orientations
        self.colors = colors
        self.cube_sizes = cube_sizes
        
    

        
    
class CubeData:
    def __init__(self, group_name, link_name):
        self.group_name = group_name
        self.link_name = link_name
        self.position, self.orientation, self.color, self.size = self.get_cube_pose()
        
    
    def get_cube_pose(self):
        cube_description_param = rospy.get_param('/' + self.group_name + '/cube_description')
        root = ET.fromstring(cube_description_param)
        
        position = None
        orientation = None
        color = None
        size = None
        
        for link in root.findall('link'):
            if link.attrib['name'] == self.link_name:
                origin = link.find('visual').find('origin')
                xyz = origin.attrib['xyz']
                rpy = origin.attrib['rpy']
                position = [float(x) for x in xyz.split()]
                orientation = [float(r) for r in rpy.split()]
                
                material = link.find('visual').find('material')
                color = material.attrib.get('name', None)
                
                geometry = link.find('visual').find('geometry')
                box = geometry.find('box')
                size = [float(s) for s in box.attrib['size'].split()]
                
                break
                
        if position is None or orientation is None:
            rospy.logwarn(f"No se encontró el enlace '{self.link_name}' en el archivo URDF para el grupo '{self.group_name}'.")
        
        return position, orientation, color, size



class CubeGenerator:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander(robot_description = '/dobot/robot_description', ns = '/dobot')
        self.scene = moveit_commander.PlanningSceneInterface(ns = '/dobot')

    def generate_cubes(self, positions, orientations, sizes, colors):
        color_count = {}
        box_names = []
        for position, orientation, size, color in zip(positions, orientations, sizes, colors):
            if color not in color_count:
                color_count[color] = 1
            else:
                color_count[color] += 1

            name = f'{color}_box_{color_count[color]}'
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "world"
            p = Point(*position)
            q = quaternion_from_euler(*orientation)
            q = Quaternion(*q)
            box_pose.pose = Pose(position = p, orientation = q)

            self.scene.add_box(name, box_pose, size)

            box_names.append(name)
            self.box_names = box_names


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate cubes in Moveit.')
    parser.add_argument('--launch_file', type=str, default = 'dobot_cubes_sim.launch', help='Name of the launch file')
    args = parser.parse_args()
    
    filename = args.launch_file

    # Construir la ruta completa al archivo dentro de la carpeta 'launch'
    current_dir = os.path.dirname(os.path.abspath(__file__))
    launch_dir = os.path.join(current_dir, '..', 'launch')
    file_path = os.path.join(launch_dir, filename)

    # Comprobar si el archivo existe
    if os.path.isfile(file_path):
        print(f"El archivo '{filename}' existe.")
        launch_file = "../launch/" + filename
        
        urdf_parser = URDFParser(launch_file)
        print(f"Se encontraron {urdf_parser.num_cubes} grupos 'cube_{{}}' en el archivo de lanzamiento.")
        print()
        
        if urdf_parser.num_cubes > 0:
            urdf_parser.collect_cube_data()
            cube_generator = CubeGenerator()
            cube_generator.generate_cubes(urdf_parser.positions, urdf_parser.orientations, urdf_parser.cube_sizes, urdf_parser.colors)
            
            print('Generated cubes:')
            for i in range(len(urdf_parser.positions)):
                print(f'Cube: {cube_generator.box_names[i]}; Position: {urdf_parser.positions[i]}; Orientation: {np.degrees(urdf_parser.orientations[i][2])}')
        
        
    else:
        print(f"El archivo '{filename}' no existe.")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from numpy import pi
import argparse
import warnings
import moveit_msgs.msg
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import String

from kinematics_problem import position2angle, angle2position
from auxiliar_functions import validate_joints, validate_position, validate_gripper_limit, moveit_joints_format, reduced_joints_format, position_from_pose, pose_from_position, obtain_orientation, cartesian_limits, joint_distance_to_limits, cartesian_distance_to_limits, orientation_distance_to_limits, line_points, are_points_collinear, obtain_arc_3D


class initiate_robot:
    def __init__(self, robot_description, ns = None):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('Move_robot', anonymous=True)
        self.robot_description = robot_description
        self.ns = ns
        if self.ns is None:
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
        else:
            self.robot = moveit_commander.RobotCommander(robot_description = self.robot_description, ns = self.ns)
            self.scene = moveit_commander.PlanningSceneInterface(ns = self.ns)

    def cubes_information(self):
        if self.ns is None:
            print("Impossible to execute. There are no cubes")
            return
        box_info = self.scene.get_known_object_names()
        attached_box = self.scene.get_attached_objects()
        if len(box_info)>0:
            for box_name in box_info:
                box_pose = self.scene.get_object_poses([box_name])[box_name]
                position, orientation = position_from_pose(box_pose, l=0)
                print(f"Cube: {box_name}; Position: {position}, Orientation: {np.degrees(orientation)}")

        if attached_box:
            box_name, box_info = next(iter(attached_box.items()))
            box_pose = box_info.object.pose
            position, orientation = position_from_pose(box_pose, l=0)
            print(f"Attached cube: {box_name}; Position: {position}; Orientation: {np.degrees(orientation)}")

        if len(box_info) == 0 and not attached_box:
            print("There are no cubes in the scene.")



    def attach_cube(self, box_name):
        if self.ns is None:
            print("Impossible to execute. There are no cubes.")
            return
        else:
            _, arm_position, arm_orientation = get_current_state(self.robot_description, self.ns)
            box_pose = self.scene.get_object_poses([box_name])[box_name]
            box_position, box_orientation = position_from_pose(box_pose, l=0)

            distance = np.linalg.norm(box_position - arm_position)

            if distance > 0.01:
                print("Impossible to execute. The arm is not close enough to the cube.")
                return

            if np.degrees(arm_orientation - box_orientation) > 1:
                print("Impossible to execute. The orientation of the arm must be similar to the orientation of the cube.")
                return

            group = moveit_commander.MoveGroupCommander('arm_group', robot_description = self.robot_description, ns = self.ns)
            eef_link = group.get_end_effector_link()
            grasping_group = 'hand'
            touch_links = self.robot.get_link_names(group = grasping_group)
            self.scene.attach_box(eef_link, box_name, touch_links = touch_links)
            print(f"Succesfully executed. {box_name} has been attached")


    def dettach_cube(self):
        if self.ns is None:
            print("Impossible to execute. There are no cubes.")
            return
        else:
            attached_box = self.scene.get_attached_objects()
            if not attached_box:
                print("Impossible to execute. There is no attached cube.")
                return

            else:
                box_name, box_info = next(iter(attached_box.items()))
                group = moveit_commander.MoveGroupCommander('arm_group', robot_description = self.robot_description, ns = self.ns)
                eef_link = group.get_end_effector_link()
                self.scene.remove_attached_object(eef_link, name = box_name)
                print(f"Succesfully executed. {box_name} has been dettached")



class move_arm:
    def __init__(self, robot_description, ns = None):
        self.robot_description = robot_description
        self.ns = ns
        if self.ns is None:
            self.move_group = moveit_commander.MoveGroupCommander('arm_group')
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                               moveit_msgs.msg.DisplayTrajectory,
                                                               queue_size=20)
            rospy.Subscriber('/execute_trajectory/feedback', moveit_msgs.msg.ExecuteTrajectoryActionFeedback, self.feedback_callback)

            
        else:
            self.move_group = moveit_commander.MoveGroupCommander('arm_group', robot_description = self.robot_description, ns = self.ns)
            self.display_trajectory_publisher = rospy.Publisher(self.ns+'/move_group/display_planned_path',
                                                               moveit_msgs.msg.DisplayTrajectory,
                                                               queue_size=20)
            rospy.Subscriber(self.ns+'/execute_trajectory/feedback', moveit_msgs.msg.ExecuteTrajectoryActionFeedback, self.feedback_callback)

        
        self.planned_joints = []
        self.planned_points = []
        self.planned_orientations = []
        self.planned_time = []
        
        self.executed_joints = []
        self.executed_points = []
        self.executed_orientations = []
        self.executed_time = []
        
        self.initial_pos = None
        self.final_pos = None
        self.intermediate_pos = None
        
        self.motion_finished = False   # Controling motion
    
    # Callback for the topic /execute_trajectory/feedback subscriber
    def feedback_callback(self, msg):
        if msg.feedback.state == "IDLE":
            self.motion_finished = True
        if msg.feedback.state == "MONITOR":
            self.motion_finished = False


    ## MOTION
    def save_planned_motion(self, plan):
        for values in plan.joint_trajectory.points:
            joint_values = list(values.positions)
            position_values, orientation_values = angle2position(*joint_values)
            time_seconds = values.time_from_start.to_sec()
 
            self.planned_joints.append(joint_values)
            self.planned_points.append(position_values)
            self.planned_orientations.append(orientation_values)
            self.planned_time.append(time_seconds)
            
    def save_executed_motion(self):
        moveit_joints = self.move_group.get_current_joint_values()
        joints = reduced_joints_format(moveit_joints)
        position, orientation = angle2position(*joints)
        
        n = len(self.executed_joints)
        
        self.executed_joints.append(joints)
        self.executed_points.append(position)
        self.executed_orientations.append(orientation)
        self.executed_time.append(n * 0.1)
        
                  
        
    def jogging_articular(self, index, step):
        increment = np.zeros(4)
        increment[index - 1] = step
        moveit_increment_joints = moveit_joints_format(increment)
        
        initial_joints, initial_position, initial_orientation = get_current_state(self.robot_description, self.ns)
        moveit_initial_joints = moveit_joints_format(initial_joints)
        moveit_final_joints = moveit_initial_joints + moveit_increment_joints
        
        final_joints = reduced_joints_format(moveit_final_joints)
        final_position, final_orientation = angle2position(*final_joints)
        
        self.initial_pos = initial_position
        self.final_pos = final_position

        success, plan, time, error = self.move_group.plan(moveit_final_joints)
        if success:
            self.save_planned_motion(plan)
            print('Initiating motion...')
            self.move_group.execute(plan, wait=False)
            
            while not self.motion_finished:
                self.save_executed_motion()
                rospy.sleep(0.1)
                            
            print("Executed motion.")           
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)
                    
    
    def jogging_cartesian(self, axis, step):
        increment = np.zeros(3)
        if axis == 'x':
            increment[0] = step
        elif axis == 'y':
            increment[1] = step
        elif axis == 'z':
            increment[2] = step
        
        initial_joints, initial_position, initial_orientation = get_current_state(self.robot_description, self.ns)
        final_position = initial_position + increment
        final_orientation = obtain_orientation(final_position, initial_joints[-1])
        
        self.initial_pos = initial_position
        self.final_pos = final_position
        
        w0 = pose_from_position(initial_position, initial_orientation)
        w1 = pose_from_position(final_position, final_orientation)
        waypoints = [w0, w1]
        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        if fraction == 1:
            self.save_planned_motion(plan)
            print('Initiating motion...')
            self.move_group.execute(plan, wait=False)
            
            while not self.motion_finished:
                self.save_executed_motion()
                rospy.sleep(0.1)
                            
            print("Executed motion.")
        else:
            warnings.warn('Motion aborted. The movement cannot be performed in a straight line.', Warning)


            
    def ptp_movj(self, final_position, final_orientation):
        _, initial_position, _ = get_current_state(self.robot_description, self.ns)
        
        self.initial_pos = initial_position
        self.final_pos = final_position
                
        final_joints = position2angle(final_position, final_orientation)
        moveit_final_joints = moveit_joints_format(final_joints)
        success, plan, time, error = self.move_group.plan(moveit_final_joints)
        if success:
            self.save_planned_motion(plan)
            print('Initiating motion...')
            self.move_group.execute(plan, wait=False)
            
            while not self.motion_finished:
                self.save_executed_motion()
                rospy.sleep(0.1)
                            
            print("Executed motion.")
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)
            
            
    def ptp_movl(self, final_position, final_orientation):
        final_joints = position2angle(final_position, final_orientation)
        _, initial_position, initial_orientation = get_current_state(self.robot_description, self.ns)
        
        self.initial_pos = initial_position
        self.final_pos = final_position

        points, orient = line_points(initial_position, final_position, initial_orientation, final_orientation, 0.005)
        waypoints = []
        for i in range(len(points)):
            pose = pose_from_position(points[i], orient[i])
            waypoints.append(pose)

        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.0025, 0.0)
        if fraction == 1:
            self.save_planned_motion(plan)
            print('Initiating motion...')
            self.move_group.execute(plan, wait=False)
                                    
            while not self.motion_finished:
                self.save_executed_motion()
                rospy.sleep(0.1)
                            
            print("Executed motion.")
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)
            
            
    def ptp_jump(self, final_position, final_orientation, height):
        _, initial_position, initial_orientation = get_current_state(self.robot_description, self.ns)
        
        self.initial_pos = initial_position
        self.final_pos = final_position

        position_1 = initial_position.copy()
        position_1[2] = height
        orientation_1 = initial_orientation

        position_2 = final_position.copy()
        position_2[2] = height
        orientation_2 = final_orientation
        
        w0 = pose_from_position(initial_position, initial_orientation)
        w1 = pose_from_position(position_1, orientation_1)
        w2 = pose_from_position(position_2, orientation_2)
        w3 = pose_from_position(final_position, final_orientation)

        waypoints = [w0, w1, w2, w3]
        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)

        if fraction == 1:
            self.save_planned_motion(plan)
            print('Initiating motion...')
            self.move_group.execute(plan, wait=False)
            
            while not self.motion_finished:
                self.save_executed_motion()
                rospy.sleep(0.1)
                            
            print("Executed motion.") 
        else:
            warnings.warn('The second movement cannot be performed in a straight line. Try with articular motion ', Warning)

            joints_1 = position2angle(position_1, orientation_1)
            joints_2 = position2angle(position_2, orientation_2)
            moveit_joints_1 = moveit_joints_format(joints_1)
            moveit_joints_2 = moveit_joints_format(joints_2)

            waypoints_1 = [w0, w1]
            plan_1, fraction_1 = self.move_group.compute_cartesian_path(waypoints_1, 0.005, 0.0)
                    
            if fraction_1 == 1:
                self.save_planned_motion(plan_1)
                print('Initiating first motion...')
                self.move_group.execute(plan_1, wait=False)
     
                while not self.motion_finished:
                    self.save_executed_motion()
                    rospy.sleep(0.1)
                            
                print("Executed first motion.")
                
                success, plan_2, time, error = self.move_group.plan(moveit_joints_2)
                        
                if success:
                    self.save_planned_motion(plan_2)
                    print('Initiating second motion...')
                    self.move_group.execute(plan_2, wait=False)
                    
                    while not self.motion_finished:
                        self.save_executed_motion()
                        rospy.sleep(0.1)
                            
                    print("Executed second motion.")

                    waypoints_2 = [w2, w3]
                    plan_3, fraction_3 = self.move_group.compute_cartesian_path(waypoints_2, 0.005, 0.0)
                            
                    if fraction_3 == 1:
                        self.save_planned_motion(plan_3)  
                        print('Initiating third motion...')
                        self.move_group.execute(plan_3, wait=False)
                        
                        while not self.motion_finished:
                            self.save_executed_motion()
                            rospy.sleep(0.1)
                                                      
                        print("Executed third motion.")
                        
                    else:
                        warnings.warn('Impossible to perform the last movement.', Warning) 
                else:
                    warnings.warn('Impossible to perform the second movement. Returning to the initial position.', Warning)
                    waypoints_1 = [w1, w0]
                    plan_1, fraction_1 = self.move_group.compute_cartesian_path(waypoints_1, 0.005, 0.0)
                    self.save_planned_motion(plan_1)                    
                    
                    self.move_group.execute(plan_1, wait=False)
                    
                    while not self.motion_finished:
                        self.save_executed_motion()
                        rospy.sleep(0.1)
            
            else:
                warnings.warn('Impossible to perform the first movement.', Warning)        
            
    
    
    
    
    def arc(self, intermediate_position, final_position, final_orientation):
        _, initial_position, initial_orientation = get_current_state(self.robot_description, self.ns)
        
        self.initial_pos = initial_position
        self.final_pos = final_position
        self.intermediate_pos = intermediate_position
        
        points = obtain_arc_3D(np.array(initial_position), np.array(intermediate_position), np.array(final_position))
        n = len(points)
        orientation = np.linspace(initial_orientation, final_orientation, n)
                        
        if np.any(points[:,2] < 0.013):
            warnings.warn('Motion aborted. The end effector will crash with the floor.', Warning)
                            
        else:
            waypoints = []
            for i in range(len(points)):
                joints = position2angle(points[i], orientation[i])
                if joints[0] is not None:
                    pose = pose_from_position(points[i], orientation[i])
                    waypoints.append(pose)
                else:
                    break
                                
            if len(waypoints) != n:
                warnings.warn('Motion aborted. The trayectory is off limits.', Warning)
                           
            else: 
                plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.0025, 0.0)
                if fraction == 1:
                    self.save_planned_motion(plan) 
                    print('Initiating motion...')
                    self.move_group.execute(plan, wait=False)
                    
                    while not self.motion_finished:
                            self.save_executed_motion()
                            rospy.sleep(0.1)
                                           
                    print("Executed motion.")
        
                else:
                    warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)    
                                        
                                        
                                        
    def creating_figures(self):
        planned_joints = np.array(self.planned_joints)
        planned_joints_deg = np.degrees(planned_joints)
        planned_positions = np.array(self.planned_points)
        planned_orientations = np.array(self.planned_orientations)
        planned_orientations_deg = np.degrees(planned_orientations)
        planned_time = np.array(self.planned_time)
        
        executed_joints = np.array(self.executed_joints)
        executed_joints_deg = np.degrees(executed_joints)
        executed_positions = np.array(self.executed_points)
        executed_orientations = np.array(self.executed_orientations)
        executed_orientations_deg = np.degrees(executed_orientations)
        executed_time = np.array(self.executed_time)
       
        print('Planned final position: ', planned_positions[-1])
        print('Executed final position: ', executed_positions[-1])
        print('Planned final orientation: ', planned_orientations_deg[-1])
        print('Executed final orientation: ', executed_orientations_deg[-1])
        print('Planned final joints: ', planned_joints_deg[-1])
        print('Executed final joints: ', executed_joints_deg[-1])
        
        plt.figure('Planned Position vs Time')
        plt.plot(planned_time, planned_positions[:,0], label=r"$x$")
        plt.plot(planned_time, planned_positions[:,1], label=r"$y$")
        plt.plot(planned_time, planned_positions[:,2], label=r"$z$")
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        #plt.ylim([-0.35, 0.35])
        plt.legend()
        plt.grid(True)
        
        plt.figure('Executed Position vs Time')
        plt.plot(executed_time, executed_positions[:,0], label=r"$x$")
        plt.plot(executed_time, executed_positions[:,1], label=r"$y$")
        plt.plot(executed_time, executed_positions[:,2], label=r"$z$")
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        #plt.ylim([-0.35, 0.35])
        plt.legend()
        plt.grid(True)

        plt.figure('YX trajectory')
        plt.plot(planned_positions[:,1], planned_positions[:,0], label="Planned trajectory")
        plt.plot(executed_positions[:,1], executed_positions[:,0], label="Executed trajectory")
        plt.ylabel('X (m)')
        plt.xlabel('Y (m)')
        plt.ylim([min(min(planned_positions[:,0]), min(executed_positions[:,0])) - 0.01, 
                  max(max(planned_positions[:,0]), max(executed_positions[:,0])) + 0.01 ])
        plt.xlim([min(min(planned_positions[:,1]), min(executed_positions[:,1])) - 0.01, 
                  max(max(planned_positions[:,1]), max(executed_positions[:,1])) + 0.01 ])
        plt.gca().invert_xaxis()
        plt.legend()
        plt.grid(True)
                            
        planned_r = np.sqrt(planned_positions[:,0]**2 + planned_positions[:,1]**2)
        executed_r = np.sqrt(executed_positions[:,0]**2 + executed_positions[:,1]**2)
        plt.figure('RZ trajectory')
        plt.plot(planned_r, planned_positions[:,2], label="Planned trajectory")
        plt.plot(executed_r, executed_positions[:,2], label="Executed trajectory")
        plt.xlabel('R (m)')
        plt.ylabel('Z (m)')
        plt.ylim([min(min(planned_positions[:,2]), min(executed_positions[:,2])) - 0.01, 
                  max(max(planned_positions[:,2]), max(executed_positions[:,2])) + 0.01 ])
        plt.xlim([min(min(planned_r), min(executed_r)) - 0.01, 
                  max(max(planned_r), max(executed_r)) + 0.01 ])
        plt.legend()
        plt.grid(True)
                        
        plt.figure('Planned Joints vs Time')
        plt.plot(planned_time, planned_joints_deg[:,0], label=r"$q_1$")
        plt.plot(planned_time, planned_joints_deg[:,1], label=r"$q_2$")
        plt.plot(planned_time, planned_joints_deg[:,2], label=r"$q_3$")
        plt.plot(planned_time, planned_joints_deg[:,3], label=r"$q_4$")
        plt.legend()
        plt.ylabel('Angles (Degrees)')
        plt.xlabel('Time (s)')
        plt.grid(True)
        #plt.ylim([-90, 90])
        
        plt.figure('Executed Joints vs Time')
        plt.plot(executed_time, executed_joints_deg[:,0], label=r"$q_1$")
        plt.plot(executed_time, executed_joints_deg[:,1], label=r"$q_2$")
        plt.plot(executed_time, executed_joints_deg[:,2], label=r"$q_3$")
        plt.plot(executed_time, executed_joints_deg[:,3], label=r"$q_4$")
        plt.legend()
        plt.ylabel('Angles (Degrees)')
        plt.xlabel('Time (s)')
        plt.grid(True)
        #plt.ylim([-90, 90])
        
        fig = plt.figure('Trajectory 3D')
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(planned_positions[:,0], planned_positions[:,1], planned_positions[:,2], linestyle='-', color='b', label="Planned trajectory")
        ax.plot(executed_positions[:,0], executed_positions[:,1], executed_positions[:,2], linestyle='-', color='r', label="Executed trajectory")
        if self.initial_pos is not None:
            ax.scatter(*self.initial_pos, color='g', label='Initial position')
        if self.final_pos is not None:
            ax.scatter(*self.final_pos, color='orange', label='Planned final position')
        if self.intermediate_pos is not None:
            ax.scatter(*self.intermediate_pos, color='m', label='Intermediate position')
        ax.scatter(*executed_positions[-1], color = 'yellow', label='Executed final position')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_xlim([min(min(planned_positions[:,0]), min(executed_positions[:,0])) - 0.01, 
                 max(max(planned_positions[:,0]), max(executed_positions[:,0])) + 0.01 ])
        ax.set_ylim([min(min(planned_positions[:,1]), min(executed_positions[:,1])) - 0.01, 
                 max(max(planned_positions[:,1]), max(executed_positions[:,1])) + 0.01 ])
        ax.set_zlim([min(min(planned_positions[:,2]), min(executed_positions[:,2])) - 0.01, 
                 max(max(planned_positions[:,2]), max(executed_positions[:,2])) + 0.01 ])
        ax.legend()
       

        plt.figure('Orientations')
        plt.plot(planned_time, planned_orientations_deg, label="Planned orientations")
        plt.plot(executed_time, executed_orientations_deg, label="Executed orientations")
        plt.legend()
        plt.ylabel('Angles (Degrees)')
        plt.xlabel('Time (s)')
        plt.grid(True)


        plt.show()
        



class move_hand:
    def __init__(self, robot_description, ns = None):
        self.robot_description = robot_description
        self.ns = ns
        if self.ns is None:
            self.move_group = moveit_commander.MoveGroupCommander('hand')
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=20)
        else:
            self.move_group = moveit_commander.MoveGroupCommander('hand', robot_description = self.robot_description, ns = self.ns)
            self.display_trajectory_publisher = rospy.Publisher(self.ns+'/move_group/display_planned_path',
                                                               moveit_msgs.msg.DisplayTrajectory,
                                                               queue_size=20)


    def open_gripper(self):
        pose_name = "opened_hand"
        self.move_group.set_named_target(pose_name)
        success, plan, time, error = self.move_group.plan()
        if success == True:
            self.move_group.execute(plan, wait = True)
            print('Successfully executed motion')
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)


    def close_gripper(self):
        pose_name = "closed_hand"
        self.move_group.set_named_target(pose_name)
        success, plan, time, error = self.move_group.plan()
        if success == True:
            self.move_group.execute(plan, wait = True)
            print('Successfully executed motion')
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)


    def move_gripper(self, distance):
        target_joints = [distance, distance]
        success,plan, time, error = self.move_group.plan(target_joints)
        if success:
            self.move_group.execute(plan, wait=True)
            print("Successfully executed motion")
        else:
            warnings.warn('Motion aborted. Failed to plan a trajectory.', Warning)


## POSITION    
def get_current_state(robot_description, ns = None):
    if ns is None:
        move_group = moveit_commander.MoveGroupCommander('arm_group')
    else:    
        move_group = moveit_commander.MoveGroupCommander('arm_group', robot_description = robot_description, ns = ns)
        
    moveit_joints = move_group.get_current_joint_values()
    joints = reduced_joints_format(moveit_joints)
    position, orientation = angle2position(*joints)
    
    return joints, position, orientation



def check_roslaunch(process_name):
    try:
        output = subprocess.check_output(['pgrep', '-f', process_name])
        return len(output.strip().splitlines()) > 0
    except subprocess.CalledProcessError:
        return False

def executing_launch_file():
    launch_files = {
        'dobot_sim.launch': ('/robot_description', None),
        'dobot_cubes_sim.launch': ('/dobot/robot_description', '/dobot')
    }

    for launch_file, (robot_description, ns) in launch_files.items():
        if check_roslaunch(launch_file):
            if launch_file == 'dobot_cubes_sim.launch':
                subprocess.run(['python3', 'spawn_cubes_moveit.py'])
            return robot_description, ns

    return None, None


def choose_action(ns):
    if ns is None:
        while True:
            action = input("Choose an action: '1' (get current state), '2' (move robot); or 'exit' to quit: ").strip()
            if action == 'exit':
                print("Exiting the program.")
                return None
            elif action in ['1', '2']:
                return action
            else:
                print("Not an action.")
    else:
        while True:
            action = input("Choose an action: '1' (get current state), '2' (move robot), '3' (action with cube); or 'exit' to quit: ").strip()
            if action == 'exit':
                print("Exiting the program.")
                return None
            elif action in ['1', '2', '3']:
                return action
            else:
                print("Not an action.")


def choose_group():
    while True:
        group = input("Choose a group: 'arm' or 'gripper'; or 'exit' to quit: ").strip()
        if group == 'exit':
            print("Exiting the program.")
            return None
        elif group in ['arm', 'gripper']:
            if group == 'arm':
                group_name = 'arm_group'
            else:     #gripper
                group_name = 'hand'
            return group_name
            #return 'arm_group' if group == 'arm' else 'hand'
        else:
            print("Not a valid group.")

####### ARM #####
def mode_arm():
    while True:
        mode = input("Choose a mode for moving the arm: '1' (JOGGING), '2' (PTP) or '3' (ARC); or 'exit' to quit: ").strip()
        if mode == 'exit':
            print("Exiting the program.")
            return None
        elif mode in ['1', '2', '3']:
            return mode
        else:
            print("Not a valid mode")




def is_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False



    ##### JOGGING #####
def jogging_submode():
    while True:
        submode = input("Choose a space: 'articular' or 'cartesian'; or 'exit' to quit: ").strip()
        if submode == 'exit':
            print('Exiting the program.')
            return None
        if submode in ['articular', 'cartesian']:
            return submode
        else:
            print("Not a valid submode.")

def jogging_articular_input(robot_description, ns):
    while True:
        index = input("Choose a joint index: '1', '2', '3', or '4'; or 'exit' to quit: ").strip()
        if index == 'exit':
            print("Exiting the program.")
            return None, None
        elif index in ['1', '2', '3', '4']:
            index = int(index)
            break
        else:
            print("Not a valid joint index.")

    joints, _, _ = get_current_state(robot_description, ns)
    to_min, to_max = joint_distance_to_limits(index, joints)

    while True:
        step = input(f"Enter the step (in degrees) between: ({to_min}, {to_max}); or 'exit' to quit: ").strip()
        if step == 'exit':
            print("Exiting the program.")
            return None, None
        elif is_float(step):
            step = float(step)
            if to_min <= step <= to_max:
                step = np.radians(step)
                break
            else:
                print(f"Step must be between ({to_min}, {to_max})")
        else:
            print("Invalid input. Enter a valid number")

    return index, step

def jogging_cartesian_input(robot_description, ns):
    while True:
        axis = input("Choose an axis: 'x', 'y' or 'z'; or 'exit' to quit: ").strip()
        if axis == 'exit':
            print("Exiting the program.")
            return None, None
        elif axis in ['x', 'y', 'z']:
            break
        else:
            print("Not a valid axis.")

    _, position, _ = get_current_state(robot_description, ns)
    to_min, to_max = cartesian_distance_to_limits(axis, position)

    while True:
        step = input(f"Enter the step (in meters) between: ({to_min}, {to_max}); or 'exit' to quit: ").strip()
        if step == 'exit':
            print("Exiting the program.")
            return None, None
        elif is_float(step):
            step = float(step)
            if to_min <= step <= to_max:
                break
            else:
                print(f"Step must be between ({to_min}, {to_max})")
        else:
            print("Invalid input. Enter a valid number")

    return axis, step

def jogging_input(robot_description, ns):
    submode = jogging_submode()
    if submode is None:
        return None, None, None
    elif submode == 'articular':
        axis, step = jogging_articular_input(robot_description, ns)
        if axis is None:
            return None, None, None
    else:   #cartesian
        axis, step = jogging_cartesian_input(robot_description, ns)
        if axis is None:
            return None, None, None
    return submode, axis, step



    ##### PTP #####
def ptp_input(robot_description, ns):
    while True:
        submode = input("Choose a submode: 'movj', 'movl' or 'jump'; or 'exit' to quit: ").strip()
        if submode == 'exit':
            print("Exiting the program.")
            return None, None, None, None
        elif submode in ['movj', 'movl', 'jump']:
            break
        else:
            print("Not a submode.")

    while True:
        target_point = input("Enter the target point (3 coordinates in meters); or 'exit' to quit: ").strip()
        if target_point == 'exit':
            print("Exiting the program.")
            return None, None, None, None
        target_point = target_point.split()
        if len(target_point) < 3:
            print("Not enough coordinate values.")
        elif len(target_point) > 3:
            print("More coordinates than needed.")
        else:
            if not all(is_float(coord) for coord in target_point):
                print("All coordinates must be floats.")
            else:
                target_point = [float(coord) for coord in target_point]
                if not validate_position(target_point):
                    print("Robot will crash with the floor. Change target point.")
                else:
                    target_joints = position2angle(target_point, 0)
                    if target_joints[0] is None:
                        print("Target point is off limits. Change target point.")
                    else:
                        break

    to_min, to_max = orientation_distance_to_limits(target_point)
    while True:
        target_orientation = input(f"Enter the target orientation (in degrees) between ({to_min}, {to_max}); or 'exit' to quit: ").strip()
        if target_orientation == 'exit':
            print("Exiting the program.")
            return None, None, None, None
        if is_float(target_orientation):
            target_orientation = float(target_orientation)
            if to_min <= target_orientation <= to_max:
                target_orientation = np.radians(target_orientation)
                break
            else:
                print(f"Target orientation must be between ({to_min}, {to_max})")
        else:
            print("Invalid input. Enter a valid number")


    if submode == 'jump':
        _, initial_point, _ = get_current_state(robot_description, ns)
        zmin_initial, zmax_initial = cartesian_limits('z', initial_point)
        zmin_final, zmax_final = cartesian_limits('z', target_point)

        zmin = max(zmin_initial, zmin_final)
        zmax = min(zmax_final, zmax_final)

        while True:
            height = input(f"Enter the height for the JUMP movement (in meters) between ({zmin}, {zmax}); or 'exit' to quit: ").strip()
            if height == 'exit':
                print("Exiting the program.")
                return None, None, None, None
            if is_float(height):
                height = float(height)
                if zmin <= height <= zmax:
                    break
                else:
                    print(f"Height must be between ({zmin}, {zmax})")
            else:
                print("Invalid input. Enter a valid number")

    else:
        height = 0

    return submode, target_point, target_orientation, height



    ##### ARC #####
def arc_input():
    while True:
        intermediate_point = input("Enter an intermediate point (3 coordinates in meters); or 'exit' to quit: "). strip()
        if intermediate_point == 'exit':
            print("Exiting the program.")
            return None, None, None
        intermediate_point = intermediate_point.split()
        if len(intermediate_point) < 3:
            print("Not enough coordinate values.")
        elif len(intermediate_point) > 3:
            print("More coordinates than needed.")
        else:
            if not all(is_float(coord) for coord in intermediate_point):
                print("All coordinates must be floats.")
            else:
                intermediate_point = [float(coord) for coord in intermediate_point]
                if not validate_position(intermediate_point):
                    print("Robot will crash with the floor. Change target point.")
                else:
                    intermediate_joints = position2angle(intermediate_point, 0)
                    if intermediate_joints[0] is None:
                        print("Intermediate point is off limits. Change intermediate point.")
                    else:
                        break

    while True:
        target_point = input("Enter the target point (3 coordinates in meters); or 'exit' to quit: ").strip()
        if target_point == 'exit':
            print("Exiting the program.")
            return None, None, None
        target_point = target_point.split()
        if len(target_point) < 3:
            print("Not enough coordinate values.")
        elif len(target_point) > 3:
            print("More coordinates than needed.")
        else:
            if not all(is_float(coord) for coord in target_point):
                print("All coordinates must be floats.")
            else:
                target_point = [float(coord) for coord in target_point]
                if not validate_position(target_point):
                    print("Robot will crash with the floor. Change target point.")
                else:
                    target_joints = position2angle(target_point, 0)
                    if target_joints[0] is None:
                        print("Target point is off limits. Change target point.")
                    else:
                        break

    to_min, to_max = orientation_distance_to_limits(target_point)
    while True:
        target_orientation = input(f"Enter the target orientation (in degrees) between ({to_min}, {to_max}); or 'exit' to quit: ").strip()
        if target_orientation == 'exit':
            print("Exiting the program.")
            return None, None, None
        if is_float(target_orientation):
            target_orientation = float(target_orientation)
            if to_min <= target_orientation <= to_max:
                target_orientation = np.radians(target_orientation)
                break
            else:
                print(f"Target orientation must be between ({to_min}, {to_max})")
        else:
            print("Invalid input. Enter a valid number")

    return intermediate_point, target_point, target_orientation




##### HAND #####
def mode_hand():
    while True:
        mode = input("Choose a mode for moving the gripper: '1' (FULLY OPENED), '2' (FULLY CLOSED) or '3' (OPEN A DISTANCE); or 'exit' to quit: ").strip()
        if mode == 'exit':
            print("Exiting the program.")
            return None
        elif mode in ['1', '2', '3']:
            return mode
        else:
            print("Not a valid mode")


def open_distance_input():
    while True:
        distance = input("Enter a distance for the jaws to open (in meters) between (0, 0.02); or 'exit' to quit: ").strip()
        if distance == 'exit':
            print("Existing the program.")
            return None
        if is_float(distance):
            distance = float(distance)
            if 0 <= distance <= 0.02:
                return distance
            else:
                print("Distance must be (in meters) between (0, 0.02).")
        else:
            print("Invalid input. Enter a valid number.")



# CUBE
def choose_cube_action():
    while True:
        action = input("Choose an action with a cube: '1' (Get cubes information), '2' (Attach cube) or '3' (Dettach cube); or 'exit' to quit: ").strip()
        if action == 'exit':
            print('Exiting the program')
            return None
        elif action in ['1', '2', '3']:
            return action
        else:
            print('Not a valid action.')


def attach_cube_input(scene):
    box_info = scene.get_known_object_names()
    if len(box_info) == 0:
        print("Impossible to execute. No cubes in the scene.")
        return 1

    attached_box = scene.get_attached_objects()
    if attached_box:
        box_name, box_info = next(iter(attached_objects.items()))
        print(f"The cube {Box_name} is already attached to the gripper. Dettach it before attaching a new one")
        return 2

    while True:
        box_name = input(f"Choose a cube name from {box_info}; or 'exit' to quit: ").strip()
        if box_name == 'exit':
            print("Exiting the program.")
            return None
        if box_name in box_info:
            return box_name
        else:
            print("Not a valid cube name.")


def main():
    robot_description, ns = executing_launch_file()
    if robot_description is None:
        print('No launch file is executed.')

    else:
        robot = initiate_robot(robot_description, ns)

        while True:
            action = choose_action(ns)
            if action is None:
                break   # Quit


            elif action == '1':  # Get current state
                joints, position, orientation = get_current_state(robot_description, ns)
                print('Initial joint state: ', np.degrees(joints))
                print('Initial position: ', position)
                print('Initial orientation: ', np.degrees(orientation))

            


            elif action == '2':  # Move robot
                group_name = choose_group()

                if group_name is None:
                    break  # Quit

                elif group_name == 'arm_group':
                    arm = move_arm(robot_description, ns)
                    mode = mode_arm()

                    if mode is None:
                        break  # Quit

                    elif mode == '1':  # JOGGING
                        submode, axis, step = jogging_input(robot_description, ns)

                        if submode is None:
                            break  # Quit
                        elif submode == 'articular':
                            arm.jogging_articular(axis, step)
                        else:   # cartesian
                            arm.jogging_cartesian(axis, step)

                    elif mode == '2':  # PTP
                        submode, target_point, target_orientation, height = ptp_input(robot_description, ns)

                        if submode is None:
                            break # Quit
                        elif submode == 'movj':
                            arm.ptp_movj(target_point, target_orientation)
                        elif submode == 'movl':
                            arm.ptp_movl(target_point, target_orientation)
                        else: # JUMP
                            arm.ptp_jump(target_point, target_orientation, height)

                    else: # ARC
                        intermediate_point, final_point, final_orientation = arc_input()

                        if intermediate_point is None:
                            break
                        else:
                            arm.arc(intermediate_point, final_point, final_orientation)


                    if len(arm.executed_joints) != 0:
                        arm.creating_figures()


                else: # Hand
                    gripper = move_hand(robot_description, ns)
                    mode = mode_hand()

                    if mode is None:
                        break

                    elif mode == '1': # FULLY OPENED
                        gripper.open_gripper()

                    elif mode == '2': # FULLY CLOSED
                        gripper.close_gripper()

                    else:   # OPEN A DISTANCE
                        distance = open_distance_input()
                        if distance is None:
                            break
                        else:
                            gripper.move_gripper(distance)

            else: # Action with cube
                cube_action = choose_cube_action()
                if cube_action is None:
                    break # Quit

                elif cube_action == '1':   # Get cubes information
                    robot.cubes_information()

                elif cube_action == '2':   # Attach cube
                    box_name = attach_cube_input(robot.scene)
                    if box_name is None:
                        break # Quit

                    elif box_name == 1 or box_name == 2:
                        pass

                    else:
                        robot.attach_cube(box_name)

                else: # Dettach cube
                    robot.dettach_cube()



if __name__ == "__main__":
    main()

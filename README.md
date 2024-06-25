# Proyecto de Manipulador Robótico con ROS y MoveIt

Este proyecto proporciona la descripción del robot Dobot Magician y configuraciones para su simulación en Gazebo y su control utilizando MoveIt y ROS.

Desarrollado por Darío Chamizo Hermosilla como Trabajo de Fin de Grado (TFG) titulado "Creación de Gemelos Digitales para la simulación y control de manipuladores robóticos o robots móviles" para la Universidad Complutense de Madrid (UCM).

## Paquetes Incluidos

### 1. dobot_description

#### Descripción

El paquete `dobot_description` contiene la descripción del modelo del robot Dobot para su uso en ROS.

#### Requisitos

Asegúrese de tener instalados los siguientes paquetes antes de usar `dobot_description`:

- roscpp
- rospy
- std_msgs
- cv_bridge
- sensor_msgs
- controller_manager
- gazebo_ros
- joint_state_publisher
- robot_state_publisher
- rqt_joint_trajectory_controller
- rviz
- urdf
- xacro


### 2. moveit_dobot

#### Descripción

El paquete `moveit_dobot` proporciona configuraciones y archivos de lanzamiento para usar el robot Dobot Magician con el framework de planificación de movimiento MoveIt.

#### Requisitos

Asegúrese de tener instalados los siguientes paquetes antes de usar `moveit_dobot`:

- moveit_ros_move_group
- moveit_fake_controller_manager
- moveit_kinematics
- moveit_planners
- moveit_ros_visualization
- moveit_setup_assistant
- moveit_simple_controller_manager
- joint_state_publisher
- joint_state_publisher_gui
- robot_state_publisher
- rviz
- tf2_ros
- xacro
- dobot_description (debe estar instalado y disponible)



## Instalación

1. Clona el repositorio en tu espacio de trabajo de ROS:

   cd ~/catkin_ws/src
   git clone https://github.com/DChamizoHermosilla/dobot_magician_DT.git


2. Compila tu espacio de trabajo de ROS usando catkin_make:
   
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash


## Uso

Para la simulación de movimientos del robot Dobot Magician siga estos pasos (a continuación se indican secuencialmente los pasos y los comandos para ejecutarlos):

1. Inicia el ROS master

   roscore

2. Lanza Gazebo y MoveIt con los archivos de lanzamiento proporcionados:

   roslaunch moveit_dobot dobot_sim.launch

3. Corra el archivo controlador de movimientos:
 
   rosrun moveit_dobot control_dobot.py


Adicionalmente, se pueden incluir cubos de colores a la simulación:

1. Inicia el ROS master

   roscore

2. Genere los cubos mediante el archivo:

   rosrun dobot_description spawn_cubes.py

3. Lanza Gazebo y Moveit con el nuevo archivo generado:

   roslaunch moveit_dobot dobot_cubes_sim.launch

4. Corra el archivo controlador de movimientos
 
   rosrun moveit_dobot control_dobot.py

5. Si lo desea, una vez cerrados los archivos, elimine los archivos de los cubos mediante:

   rosrun dobot_description delete_cube_files.py



## Explicaciones Adicionales

Este proyecto a tomado como referencias los trabajos:

- magician_dobot: https://github.com/JimEverest/magician_dobot/tree/master

- polman_ros_dobot: https://github.com/nj-ramadhan/polman-ros-dobot




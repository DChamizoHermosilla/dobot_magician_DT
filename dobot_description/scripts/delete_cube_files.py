import os

def delete_files():
    # Directorios y patrones de archivos a eliminar
    urdf_directory = '../urdf'
    launch_directory = '../launch'
    moveit_launch_directory = '../../moveit_dobot/launch'

    urdf_pattern = 'cube*.urdf'
    launch_pattern = 'spawn_cube*.launch'
    world_launch_file = 'world.launch'
    moveit_launch_file = 'dobot_cubes_sim.launch'

    # Función para eliminar archivos según el patrón y directorio especificado
    def delete_files_by_pattern(directory, pattern):
        files = os.listdir(directory)
        for file in files:
            if file.startswith(pattern.split('*')[0]) and file.endswith(pattern.split('*')[1]):
                file_path = os.path.join(directory, file)
                os.remove(file_path)
                print(f"Deleted file: {file_path}")

    # Eliminar archivos URDF en urdf/
    delete_files_by_pattern(urdf_directory, urdf_pattern)

    # Eliminar archivos de lanzamiento en launch/
    delete_files_by_pattern(launch_directory, launch_pattern)

    # Eliminar world.launch en launch/
    world_file_path = os.path.join(launch_directory, world_launch_file)
    if os.path.exists(world_file_path):
        os.remove(world_file_path)
        print(f"Deleted file: {world_file_path}")
    else:
        print(f"File not found: {world_file_path}")

    # Eliminar dobot_cubes_sim.launch en moveit_dobot/launch/
    moveit_file_path = os.path.join(moveit_launch_directory, moveit_launch_file)
    if os.path.exists(moveit_file_path):
        os.remove(moveit_file_path)
        print(f"Deleted file: {moveit_file_path}")
    else:
        print(f"File not found: {moveit_file_path}")

if __name__ == "__main__":
    delete_files()

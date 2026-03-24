import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.expanduser('~/my_robot_car/urdf/car.urdf')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]))
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'my_car', '-file', urdf_file], output='screen')
    return LaunchDescription([gazebo, spawn_entity])

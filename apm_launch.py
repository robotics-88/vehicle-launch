import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config_yaml = os.path.join(
      get_package_share_directory('mavros'),
      'config',
      'apm_config.yaml'
      )

   return LaunchDescription([
      Node(
         package='mavros',
         executable='mavros_node',
         namespace='mavros',
         name='mavros',
         parameters=[config_yaml]
      )
   ])

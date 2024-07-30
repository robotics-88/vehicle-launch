import os, sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def setup_nodes(context, *args, **kwargs):
    simulate = LaunchConfiguration('simulate').perform(context)

    if simulate.lower() in ['true', '1']:
        fcu_url = 'udp://:14551@'
    else:
        fcu_url = '/dev/cubeorange:57600'

    mavros = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_launch'),
            'launch/apm.launch')
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
    )

    return [mavros]
    


def generate_launch_description():

    ### Launch arguments
    launch_args = []

    simulate_launch_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='Run in simulation mode'
    )
    launch_args.append(simulate_launch_arg)

    return LaunchDescription(launch_args + 
        [
        OpaqueFunction(function=setup_nodes)
        ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml, os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='decco.yaml'),
        DeclareLaunchArgument('mavros_map_frame', default_value='map'),
        DeclareLaunchArgument('mavros_base_frame', default_value='base_link'),
        DeclareLaunchArgument('slam_base_frame', default_value='livox_frame'),
        DeclareLaunchArgument('slam_map_frame', default_value='slam_map'),
        DeclareLaunchArgument('slam_pose_topic', default_value='/decco/pose'),
        DeclareLaunchArgument('perception_file', default_value=os.path.join(get_package_share_directory('vehicle_launch'), 'config/perception.json')),
        DeclareLaunchArgument('do_airsim', default_value='false'),
        DeclareLaunchArgument('offline', default_value='false'),
        DeclareLaunchArgument('save_pcd', default_value='false'),
        DeclareLaunchArgument('do_record', default_value='true'),
        DeclareLaunchArgument('cloud_registered_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('cloud_stabilized', default_value='/cloud_registered_map'),
        DeclareLaunchArgument('cloud_aggregated', default_value='/cloud_aggregated'),
        DeclareLaunchArgument('record_config_file', default_value=os.path.join(get_package_share_directory('vehicle_launch'), 'config/r88_default.config')),
        DeclareLaunchArgument('data_directory', default_value='/home/$(env USER)/r88_public/records/'),
        DeclareLaunchArgument('default_alt', default_value='3.0'),
        DeclareLaunchArgument('min_alt', default_value='1.0'),
        DeclareLaunchArgument('max_alt', default_value='5.0'),
        DeclareLaunchArgument('planning_horizon', default_value='6.0'),
        DeclareLaunchArgument('velocity_setpoint_speed', default_value='0.5'),
        DeclareLaunchArgument('setpoint_acceptance_radius', default_value='0.5'),
        DeclareLaunchArgument('goal_acceptance_radius', default_value='2.0'),
        DeclareLaunchArgument('obstacle_dist_threshold', default_value='2.0'),
        DeclareLaunchArgument('adjust_altitude_volume', default_value='true'),
        DeclareLaunchArgument('criteria', default_value='DIST,INFO,SAFETY'),
        DeclareLaunchArgument('weights', default_value='0.2,0.1,0.7'),
        DeclareLaunchArgument('dem_name', default_value='test.tif'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_raw'),
        OpaqueFunction(function=launch_from_config)
    ])

def launch_from_config(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    config_path = os.path.join(get_package_share_directory('vehicle_launch'), 'config/vehicles', config_file)
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    drone_id = cfg.get('drone_id', 'drone')
    frame_id = cfg.get('frame_id', 'base_link')
    simulated = cfg.get('simulated', False)
    flight_controller = cfg.get('flight_controller', 'ardupilot')
    sensors = cfg.get('sensors', {})
    rviz = False
    do_slam = False
    do_airsim = LaunchConfiguration('do_airsim').perform(context).lower() == 'true'

    base_link_height = 0.23

    # flight controller params
    flight_controller = cfg.get('flight_controller', {})
    fcu_type = flight_controller.get('type', 'ardupilot')
    fcu_url = flight_controller.get('fcu_url', '/dev/cubeorange:57600')

    # LiDAR params for transform/slam handling
    lidar_cfg = sensors.get('lidar_top')
    slam_config_file = 'mid360.yaml'
    if lidar_cfg:
        do_slam = True
        slam_config_file = lidar_cfg.get('type', 'mid360') + '.yaml'
        lidar_pos = lidar_cfg.get('position', [0.0, 0.0, 0.0])
        lidar_rpy = lidar_cfg.get('orientation_rpy', [0.0, 0.0, 0.0])
        lidar_x = str(lidar_pos[0])
        lidar_z = str(lidar_pos[2])
        lidar_pitch = str(lidar_rpy[1])  # pitch = Y rotation
    else:
        lidar_x = '0.0'
        lidar_z = '0.0'
        lidar_pitch = '0.0'

    nodes = []

    # Simulation bridge
    if simulated:
        rviz = True
        base_link_height = 0.21
        if do_airsim:
            airsim_launch = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('airsim_launch'), 'launch/airsim.xml')),
                launch_arguments={
                    'do_slam': str(do_slam).lower(),
                    'enable_cameras': 'true',
                    'vehicle_name': drone_id,
                    'vehicle_frame': frame_id,
                }.items()
            )
            nodes.append(airsim_launch)
        else:
            gazebo_launch = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_launch'), 'launch/ros_gz_bridge.launch'))
            )
            nodes.append(gazebo_launch)

    # SLAM
    if do_slam:
        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('fast_lio'), 'launch/mapping.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'config_file': slam_config_file
            }.items()
        )
        nodes.append(slam_launch)

    # Rviz
    if rviz:
        rviz_config_file = os.path.join(get_package_share_directory('vehicle_launch'), 'config', 'airsim.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': simulated}]
        )
        nodes.append(rviz_node)

    # Static transform publisher for map to world frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '-' + str(base_link_height),  # Translation (x, y, z)
            '0', '0', '0',  # Rotation (roll, pitch, yaw)
            LaunchConfiguration('mavros_map_frame'), 'world'  # Parent frame and child frame
        ],
        name='map_world_tf'
    )
    nodes.append(static_tf_node)

    # MAVROS
    if fcu_type == 'ardupilot':
        apm_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_launch'), 'launch/apm.launch')),
            launch_arguments={
                'do_slam': str(do_slam).lower(),
                'fcu_url': fcu_url,
            }.items()
        )
        nodes.append(apm_launch)
    elif fcu_type == 'px4':
        px4_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_launch'), 'launch/px4.launch')),
            launch_arguments={
                'do_slam': str(do_slam).lower(),
                'fcu_url': fcu_url,
            }.items()
        )
        nodes.append(px4_launch)

    # Static TFs and sensor nodes
    for name, sensor in sensors.items():
        pos = sensor.get('position', [0, 0, 0])
        rpy = sensor.get('orientation_rpy', [0, 0, 0])
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(pos[0]), str(pos[1]), str(pos[2]),
                str(rpy[0]), str(rpy[1]), str(rpy[2]),
                sensor['frame'], frame_id
            ],
            name=f"{name}_tf"
        )
        nodes.append(tf_node)

        sensor_launch_file = os.path.join(
            get_package_share_directory('vehicle_launch'), 'launch/sensors', f"{sensor['type']}.launch"
        )
        if os.path.exists(sensor_launch_file):
            nodes.append(IncludeLaunchDescription(
                XMLLaunchDescriptionSource(sensor_launch_file)
            ))

    # Task Manager
    task_manager_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('task_manager'), 'launch/task_manager.launch')),
        launch_arguments={
            'mavros_map_frame': LaunchConfiguration('mavros_map_frame'),
            'base_frame': LaunchConfiguration('mavros_base_frame'),
            'slam_map_frame': LaunchConfiguration('slam_map_frame'),
            'enable_autonomy': 'true',
            'use_failsafes': 'false',
            'slam_pose_topic': LaunchConfiguration('slam_pose_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'do_slam': str(do_slam).lower(),
            'lidar_topic': sensors.get('lidar_top', {}).get('topic', '/lidar'),
            'mapir_topic': sensors.get('camera_front', {}).get('topic', '/image_raw'),
            'do_record': LaunchConfiguration('do_record'),
            'record_config_file': LaunchConfiguration('record_config_file'),
            'perception_file': LaunchConfiguration('perception_file'),
            'data_directory': LaunchConfiguration('data_directory'),
            'default_alt': LaunchConfiguration('default_alt'),
            'min_alt': LaunchConfiguration('min_alt'),
            'max_alt': LaunchConfiguration('max_alt'),
            'simulate': str(simulated).lower(),
            'offline': LaunchConfiguration('offline'),
            'lidar_pitch': lidar_pitch,
            'lidar_x': lidar_x,
            'lidar_z': lidar_z
        }.items()
    )
    nodes.append(task_manager_launch)

    return nodes

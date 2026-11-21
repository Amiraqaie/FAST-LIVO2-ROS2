import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def highrate_node(context):
    with open(LaunchConfiguration('mapper_params').perform(context)) as stream:
        config = yaml.safe_load(stream)
    params = {}
    for key in ('/**', 'laserMapping', '/laserMapping'):
        params.update(config.get(key, {}).get('ros__parameters', {}).get('imu', {}))
    if not params.get('imu_en', True):
        return []
    noise = {key: params.get(key, default) for key, default in (
        ('acc_cov', 1.0), ('gyr_cov', 1.0), ('b_acc_cov', 0.0001), ('b_gyr_cov', 0.0001))}
    return [Node(package='fast_lio', executable='highrate_odom',
                 parameters=[noise, {'imu_topic': '/laserMapping/imu_si', 'acc_scale': 1.0}],
                 output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mapper_params'),
        DeclareLaunchArgument('highrate', default_value='true'),
        OpaqueFunction(function=highrate_node, condition=IfCondition(LaunchConfiguration('highrate'))),
    ])

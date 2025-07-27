import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='example_mesh_antipodal_generator.yaml',
        description='YAML file name for grasp generator parameters.'
    )

    config_path = PathJoinSubstitution([
        FindPackageShare('graspgen_tutorials'),
        'config',
        LaunchConfiguration('config')
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('graspgen_tutorials'),
        'config',
        'grasps.rviz'
    ])

    return launch.LaunchDescription([
        config_arg,

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'object']
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='graspgen_tutorials',
            executable='mesh_graspgen_service',
            name='graspgen_server',
            parameters=[config_path, {'config_filename': LaunchConfiguration('config')}],
            output='screen'
        )
    ])
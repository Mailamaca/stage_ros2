from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare('stage_ros2'),
        # 'stage',
        'worlds',
        'willow-erratic.world'
    ])

    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='simulation',
        description='Top-level namespace')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=default_world,
        description='Full path to the stage world definition')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Nodes launching commands
    run_sim_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        output='screen',
        respawn=True,
        namespace=namespace,
        # name='stage',
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
                "world": world_file,
                "gui": True,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_world_file_cmd)

    ld.add_action(SetParameter('use_sim_time', use_sim_time))

    ld.add_action(run_sim_node)

    return ld

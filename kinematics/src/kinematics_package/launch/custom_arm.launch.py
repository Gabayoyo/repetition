from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Declare terminal arguments (with defaults)
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='px100',
        description='Robot model name (e.g., px100, px150)',
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=LaunchConfiguration('robot_model'),
        description='ROS namespace for the robot (defaults to robot_model)',
    )

    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')

    pkg_share = FindPackageShare('kinematics_package')

    # 2. Build dynamic file paths based on robot_model
    urdf_path = PathJoinSubstitution([
        pkg_share, 'urdf', [robot_model, '.urdf.xacro']
    ])
    motor_yaml_path = PathJoinSubstitution([
        pkg_share, 'config', robot_model, 'motor.yaml'
    ])
    mode_yaml_path = PathJoinSubstitution([
        pkg_share, 'config', robot_model, 'mode.yaml'
    ])
    rviz_config_path = PathJoinSubstitution([
        pkg_share, 'config', robot_model, [robot_model, '.rviz']
    ])

    # 3. Compile Xacro dynamically
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_path,
        ' robot_name:=', robot_name,
        ' base_link_frame:=base_link',
    ])

    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'robot_description': robot_description_content,
            'motor_configs': motor_yaml_path,
            'mode_configs': mode_yaml_path,
            'use_rviz': 'false',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name,
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    arm_controller_node = Node(
        package='kinematics_package',
        executable='arm_controller',
        name='arm_controller',
        parameters=[{'robot_name': robot_name}],
        output='screen',
    )

    return LaunchDescription([
        robot_model_arg,
        robot_name_arg,
        xsarm_control_launch,
        rviz_node,
        arm_controller_node,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ur10e_rviz_control')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur10e.urdf')
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    joy_script = os.path.join(pkg_share, 'scripts', 'dual_control.py')

    return LaunchDescription([
        # Публикатор состояния робота
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        # Публикатор состояний суставов (если не используется аппаратный интерфейс)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states']}],
            output='screen'
        ),

        # Узел джойстика
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_name': 'DualSense Wireless Controller'}],
            output='screen'
        ),

        # Ваш кастомный узел управления
        Node(
            package='ur10e_rviz_control',
            executable='dual_control.py',
            name='dual_control',
            output='screen',
            prefix='python3'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Статический TF-публикатор для ft_frame (если не определено в URDF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_ft_frame',
            arguments=['0', '0', '0.1', '0', '0', '0', 'wrist_3_link', 'ft_frame'],
            output='screen'
        )
    ])

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('self_balancing_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xml')
    world_file = os.path.join(pkg_path, 'worlds', 'balancing.sdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Start Gazebo with custom world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # Publish robot description for RViz and Gazebo
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Spawn robot after 5 seconds
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'self_balancing_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.4',
                ],
                output='screen'
            ),
        ]),

        # ROS-Gazebo Bridge: connects ROS 2 topics with Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            output='screen'
        ),

    ])

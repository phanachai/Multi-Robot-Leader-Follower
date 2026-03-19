from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    follower_robot2 = Node(
        package='follower_turtlebot_leader',
        executable='follower_node',
        name='follower_robot2',
        output='screen',
        parameters=[
            {'target_robot': 'robot1'},
            {'my_robot': 'robot2'}
        ]
    )

    follower_robot3 = Node(
        package='follower_turtlebot_leader',
        executable='follower_node',
        name='follower_robot3',
        output='screen',
        parameters=[
            {'target_robot': 'robot2'},
            {'my_robot': 'robot3'}
        ]
    )

    return LaunchDescription([
        follower_robot2,
        follower_robot3
    ])

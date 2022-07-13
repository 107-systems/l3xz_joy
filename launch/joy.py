from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_joy',
      executable='l3xz_joy_node',
      name='joy',
      namespace='l3xz',
      output='screen',
      parameters=[
        {'joy_dev_node': '/dev/input/js0'},
        {'joy_topic': 'joy'},
      ]
    )
  ])

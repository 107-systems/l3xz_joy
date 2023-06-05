from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_joy',
      executable='l3xz_joy_node',
      name='l3xz_joy',
      namespace='l3xz',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'joy_dev_node': '/dev/input/js0'},
        {'joy_topic': 'joy'},
        {'joy_topic_publish_period_ms': 50},
        { 'joy_deadzone': 0.01 },
      ]
    )
  ])

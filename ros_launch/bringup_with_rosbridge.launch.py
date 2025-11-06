"""
Example launch file that adds rosbridge_websocket and web_video_server
to your existing bringup.launch.py

To use: Copy the rosbridge and web_video_server nodes into your actual bringup.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TODO: Add your existing bringup nodes here
        # ... (keep your existing nodes)
        
        # Add rosbridge_websocket for frontend connection
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',  # Listen on all interfaces
            }],
            output='screen'
        ),
        
        # Add web_video_server for camera streaming
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
            }],
            output='screen'
        ),
    ])


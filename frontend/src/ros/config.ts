export const ROS_CONFIG = {
  rosbridgeUrl: import.meta.env.VITE_ROSBRIDGE_URL || 'ws://fordward.local:9090',
  videoBase: import.meta.env.VITE_VIDEO_BASE || 'http://fordward.local:8080',
  topics: {
    cmdVel: '/cmd_vel',
    odom: '/odom',
    battery: '/ros_robot_controller/battery',
    camera: '/ascamera/camera_publisher/rgb0/image',
  },
  messageTypes: {
    cmdVel: 'geometry_msgs/Twist',
    odom: 'nav_msgs/Odometry',
    battery: 'std_msgs/Float32', // Adjust based on actual type
    camera: 'sensor_msgs/Image',
  }
};


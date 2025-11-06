export const ROS_CONFIG = {
  rosbridgeUrl: import.meta.env.VITE_ROSBRIDGE_URL || 'ws://fordward.local:9090',
  videoBase: import.meta.env.VITE_VIDEO_BASE || 'http://fordward.local:8080',
  topics: {
    cmdVel: '/cmd_vel',
    odom: '/odom',
    battery: '/ros_robot_controller/battery',
    camera: '/ascamera/camera_publisher/rgb0/image',
    map: '/map',
    mapMetadata: '/map_metadata',
    pois: '/pois',
  },
  services: {
    changeMap: '/change_map',
    listMaps: '/list_maps',
  },
  actions: {
    navigateToPose: '/navigate_to_pose',
  },
  messageTypes: {
    cmdVel: 'geometry_msgs/Twist',
    odom: 'nav_msgs/Odometry',
    battery: 'std_msgs/UInt16', // Battery topic uses UInt16
    camera: 'sensor_msgs/Image',
    map: 'nav_msgs/OccupancyGrid',
    mapMetadata: 'nav_msgs/MapMetaData',
    pois: 'std_msgs/String', // Adjust based on actual POI message type
  }
};


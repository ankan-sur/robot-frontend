const defaultHost = typeof window !== 'undefined' && window.location?.hostname
  ? window.location.hostname
  : 'localhost';

const defaultRosbridge = `ws://${defaultHost}:9090`;
const defaultVideoBase = `http://${defaultHost}:8080`;
// Rosboard and RWT URLs can be added via env if needed

export const ROS_CONFIG = {
  rosbridgeUrl: import.meta.env.VITE_ROSBRIDGE_URL || import.meta.env.VITE_ROSBRIDGE_FALLBACK_URL || defaultRosbridge,
  videoBase: import.meta.env.VITE_VIDEO_BASE || import.meta.env.VITE_VIDEO_FALLBACK_BASE || defaultVideoBase,
  topics: {
    cmdVel: '/cmd_vel',
    odom: '/odom',
    battery: '/ros_robot_controller/battery',
    camera: import.meta.env.VITE_CAMERA_TOPIC || '/ascamera/camera_publisher/rgb0/image',
    map: '/map',
    mapMetadata: '/map_metadata',
    pois: '/pois',
    availableMaps: '/available_maps',
    imuRpy: '/imu/rpy/filtered',
    jointStates: '/joint_states',
    diagnostics: '/diagnostics',
    button: '/ros_robot_controller/button',
    robotState: '/robot/state', // 'idle', 'responding_to_command', 'heading_to_charger'
    rosout: '/rosout',
  },
  services: {
    systemMapSelect: '/system/map/select',
    systemMapPois: '/system/map/pois',
  },
  actions: {
    navigateToPose: '/navigate_to_pose',
  },
  messageTypes: {
    cmdVel: 'geometry_msgs/Twist',
    odom: 'nav_msgs/Odometry',
    battery: 'std_msgs/UInt16', // Battery topic uses UInt16 (adjust if Float32)
    camera: 'sensor_msgs/Image',
    map: 'nav_msgs/OccupancyGrid',
    mapMetadata: 'nav_msgs/MapMetaData',
    availableMaps: 'std_msgs/String',
    // POIs now published by system_topics as interfaces/Points (fallbacks handled in hooks)
    pois: 'interfaces/Points',
    imuRpy: 'geometry_msgs/Vector3Stamped', // Roll, Pitch, Yaw
    jointStates: 'sensor_msgs/JointState',
    diagnostics: 'diagnostic_msgs/DiagnosticArray',
    button: 'ros_robot_controller_msgs/ButtonState',
    robotState: 'std_msgs/String', // Robot state string
    rosout: 'rcl_interfaces/Log',
  }
};

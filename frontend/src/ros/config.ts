const defaultHost = typeof window !== 'undefined' && window.location?.hostname
  ? window.location.hostname
  : 'localhost';

const defaultRosbridge = `ws://${defaultHost}:9090`;
const defaultVideoBase = `http://${defaultHost}:8080`;
const defaultRwtBase = `http://${defaultHost}:8001`;
const defaultRosboard = `http://${defaultHost}:8081`;

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
    imuRpy: '/imu/rpy/filtered',
    jointStates: '/joint_states',
    diagnostics: '/diagnostics',
    button: '/ros_robot_controller/button',
    robotState: '/robot/state', // 'idle', 'responding_to_command', 'heading_to_charger'
    rosout: '/rosout',
  },
  services: {
    changeMap: '/change_map',
    listMaps: '/list_maps',
  },
  actions: {
    navigateToPose: '/navigate_to_pose',
  },
  rwt: {
    map: import.meta.env.VITE_RWT_MAP_URL || `${import.meta.env.VITE_RWT_BASE || defaultRwtBase}/rwt/map`,
    teleop: import.meta.env.VITE_RWT_TELEOP_URL || `${import.meta.env.VITE_RWT_BASE || defaultRwtBase}/rwt/teleop`,
    image: import.meta.env.VITE_RWT_IMAGE_URL || `${import.meta.env.VITE_RWT_BASE || defaultRwtBase}/rwt/image`,
    rosboard: import.meta.env.VITE_ROSBOARD_URL || defaultRosboard,
  },
  messageTypes: {
    cmdVel: 'geometry_msgs/Twist',
    odom: 'nav_msgs/Odometry',
    battery: 'std_msgs/UInt16', // Battery topic uses UInt16 (adjust if Float32)
    camera: 'sensor_msgs/Image',
    map: 'nav_msgs/OccupancyGrid',
    mapMetadata: 'nav_msgs/MapMetaData',
    pois: 'std_msgs/String', // Adjust based on actual POI message type
    imuRpy: 'geometry_msgs/Vector3Stamped', // Roll, Pitch, Yaw
    jointStates: 'sensor_msgs/JointState',
    diagnostics: 'diagnostic_msgs/DiagnosticArray',
    button: 'ros_robot_controller_msgs/ButtonState',
    robotState: 'std_msgs/String', // Robot state string
    rosout: 'rcl_interfaces/Log',
  }
};

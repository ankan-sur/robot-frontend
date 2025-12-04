const defaultHost = typeof window !== 'undefined' && window.location?.hostname
  ? window.location.hostname
  : 'localhost';

// Connection mode: 'direct' (rosbridge) or 'cloud' (via cloud backend)
export const CONNECTION_MODE = import.meta.env.VITE_CONNECTION_MODE || 'direct';

// For cloud mode, don't auto-connect to rosbridge
const defaultRosbridge = CONNECTION_MODE === 'cloud' 
  ? '' // Empty - don't connect directly
  : `ws://${defaultHost}:9090`;
  
const defaultVideoBase = `http://${defaultHost}:8080`;

// Cloud backend URL (for cloud mode)
export const CLOUD_BACKEND_URL = import.meta.env.VITE_CLOUD_BACKEND_URL || 'wss://hfhbackend.onrender.com/ui';
export const ROBOT_ID = import.meta.env.VITE_ROBOT_ID || 'fordward';

export const ROS_CONFIG = {
  rosbridgeUrl: import.meta.env.VITE_ROSBRIDGE_URL || import.meta.env.VITE_ROSBRIDGE_FALLBACK_URL || defaultRosbridge,
  videoBase: import.meta.env.VITE_VIDEO_BASE || import.meta.env.VITE_VIDEO_FALLBACK_BASE || defaultVideoBase,
  topics: {
    cmdVel: '/ui/cmd_vel',
    odom: '/odom',
    battery: '/ros_robot_controller/battery',
    camera: import.meta.env.VITE_CAMERA_TOPIC || '/ascamera/camera_publisher/rgb0/image',
    map: '/map',
    mapMetadata: '/map_metadata',
    pois: '/pois',
    availableMaps: '/available_maps',
  // modeStatus removed (legacy SLAM manager)
    imuRpy: '/imu/rpy/filtered',
    jointStates: '/joint_states',
    diagnostics: '/diagnostics',
    button: '/ros_robot_controller/button',
  // Prefer canonical '/robot/state' (used by system_topics). Keep legacy
  // '/robot_state' as a fallback where needed.
  robotState: '/robot/state', // 'idle', 'responding_to_command', 'heading_to_charger'
    rosout: '/rosout',
  },
  services: {
    // New service-based control (provided by backend ROS node)
    setMode: '/set_mode', // interfaces/SetString with data: arbitrary mode string (e.g. 'slam','localization','idle')
    getMode: '/get_mode', // std_srvs/Trigger, message field contains JSON like {"mode":"localization","map":"<name>"}
    stopSlamAndSave: '/stop_slam_and_save', // interfaces/SetString with data: map name
    loadMap: '/load_map', // interfaces/SetString with data: map name
    listMaps: '/list_maps', // returns JSON list (string) or an array field
    // getMode/startSlam/stopSlamAndSave removed (legacy SLAM service endpoints)
    navCancel: '/navigate_to_pose/cancel',
  // Teleop safety
  teleopEStop: '/teleop/estop',
  teleopClearEStop: '/teleop/clear_estop',
  systemMapRefresh: '/system/map/refresh',
    // Legacy/system services (still supported for fallback)
    systemMapSelect: '/system/map/select',
    systemMapInfo: '/system/map/info',
    systemMapPois: '/system/map/pois',
    
    // POI management services
    markPoi: '/system/map/mark_poi',      // interfaces/SetString - add/update POI
    deletePoi: '/system/map/delete_poi',  // interfaces/SetString - remove POI
    renamePoi: '/system/map/rename_poi',  // interfaces/SetString - rename POI
  },
  actions: {
    navigateToPose: '/navigate_to_pose',
  },
  // Navigation status
  nav: {
    statusTopic: '/navigate_to_pose/status'
  },
  messageTypes: {
    cmdVel: 'geometry_msgs/Twist',
    odom: 'nav_msgs/Odometry',
    battery: 'std_msgs/UInt16', // Battery topic uses UInt16 (adjust if Float32)
    camera: 'sensor_msgs/Image',
    map: 'nav_msgs/OccupancyGrid',
    mapMetadata: 'nav_msgs/MapMetaData',
    availableMaps: 'std_msgs/String',
  // modeStatus removed
    // POIs now published by system_topics as interfaces/Points (fallbacks handled in hooks)
    pois: 'interfaces/Points',
    imuRpy: 'geometry_msgs/Vector3Stamped', // Roll, Pitch, Yaw
    jointStates: 'sensor_msgs/JointState',
    diagnostics: 'diagnostic_msgs/DiagnosticArray',
    button: 'ros_robot_controller_msgs/ButtonState',
    robotState: 'std_msgs/String', // Robot state string
    rosout: 'rcl_interfaces/Log',
    navStatus: 'action_msgs/GoalStatusArray',
  }
};

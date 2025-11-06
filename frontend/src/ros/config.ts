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
    imuRpy: '/imu/rpy/filtered',
    jointStates: '/joint_states',
    diagnostics: '/diagnostics',
    button: '/ros_robot_controller/button',
    robotState: '/robot/state', // 'idle', 'responding_to_command', 'heading_to_charger'
    currentCommand: '/current_command', // Current destination/command info
    // Optional UI/bridge topics (if your ui_bridge publishes these)
    robotTelemetry: '/robot/telemetry',
    robotLog: '/robot/log',
    cmdFeedback: '/robot/cmd_feedback',
    uiCmd: '/ui/cmd',
    uiCancel: '/ui/cmd_cancel',
    rosout: '/rosout',
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
    battery: 'std_msgs/UInt16', // Battery topic uses UInt16 (adjust if Float32)
    camera: 'sensor_msgs/Image',
    map: 'nav_msgs/OccupancyGrid',
    mapMetadata: 'nav_msgs/MapMetaData',
    pois: 'std_msgs/String', // Adjust based on actual POI message type
    imuRpy: 'geometry_msgs/Vector3', // Roll, Pitch, Yaw
    jointStates: 'sensor_msgs/JointState',
    diagnostics: 'diagnostic_msgs/DiagnosticArray',
    button: 'std_msgs/Bool', // or UInt8, adjust based on actual type
    robotState: 'std_msgs/String', // Robot state string
    currentCommand: 'std_msgs/String', // Current command/destination info
    // Optional UI/bridge message types
    robotTelemetry: 'std_msgs/String',
    robotLog: 'std_msgs/String',
    cmdFeedback: 'std_msgs/String',
    uiCmd: 'std_msgs/String',
    uiCancel: 'std_msgs/String',
    rosout: 'rcl_interfaces/Log',
  }
};


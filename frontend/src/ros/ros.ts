import ROSLIB from 'roslib';
import { ROS_CONFIG } from './config';

export const ros = new ROSLIB.Ros({
  url: ROS_CONFIG.rosbridgeUrl,
  groovyCompatibility: false
} as any);


// Ensure we close the websocket cleanly when the page unloads. Browsers do not
// always complete async work on reload, but explicitly closing reduces the
// chance the server tries to write to an already-closed socket and spams errors.
if (typeof window !== 'undefined' && window.addEventListener) {
  let reconnectTimeout: number | undefined;
  
  window.addEventListener('beforeunload', () => {
    try { 
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      ros.close(); 
    } catch (e) { /* ignore */ }
  });
  
  // Add exponential backoff on disconnect to avoid flapping reconnects
  let reconnectAttempts = 0;
  const maxReconnectDelay = 10000;
  const baseDelay = 1000;
  
  ros.on('close', () => {
    if (reconnectTimeout) clearTimeout(reconnectTimeout);
    const delay = Math.min(maxReconnectDelay, baseDelay * Math.pow(1.5, reconnectAttempts));
    reconnectAttempts++;
    reconnectTimeout = window.setTimeout(() => {
      try { ros.connect(ROS_CONFIG.rosbridgeUrl); } catch (e) { /* ignore */ }
    }, delay);
  });
  
  ros.on('connection', () => {
    reconnectAttempts = 0;
    if (reconnectTimeout) clearTimeout(reconnectTimeout);
  });
}


export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';

let connectionState: ConnectionState = 'disconnected';
const listeners: Set<(state: ConnectionState) => void> = new Set();

ros.on('connection', () => {
  console.log('Connected to rosbridge');
  connectionState = 'connected';
  listeners.forEach(fn => fn(connectionState));
});

ros.on('close', () => {
  console.log('Disconnected from rosbridge');
  connectionState = 'disconnected';
  listeners.forEach(fn => fn(connectionState));
});

ros.on('error', (error) => {
  console.error('Rosbridge error:', error);
  connectionState = 'error';
  listeners.forEach(fn => fn(connectionState));
});

export function getConnectionState(): ConnectionState {
  return connectionState;
}

export function onConnectionChange(callback: (state: ConnectionState) => void): () => void {
  listeners.add(callback);
  callback(connectionState); // Call immediately with current state
  return () => listeners.delete(callback);
}

// Convenience topics map (optional): provides ready-to-use ROSLIB.Topic instances
// Only use the ones that exist in your system; others will be harmless if unused.
// Throttle high-frequency topics to reduce rosbridge load with multiple clients
export const topics = {
  // Core
  cmdVel: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.cmdVel, messageType: ROS_CONFIG.messageTypes.cmdVel }),
  odom: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.odom, messageType: ROS_CONFIG.messageTypes.odom, throttle_rate: 200, queue_length: 1 }),  // 5 Hz
  battery: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.battery, messageType: ROS_CONFIG.messageTypes.battery, throttle_rate: 2000, queue_length: 1 }),  // 0.5 Hz

  robotState: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.robotState, messageType: ROS_CONFIG.messageTypes.robotState, throttle_rate: 500, queue_length: 1 }),  // 2 Hz
  rosout: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.rosout, messageType: ROS_CONFIG.messageTypes.rosout, throttle_rate: 500, queue_length: 5 }),  // 2 Hz, keep recent logs
  navStatus: new ROSLIB.Topic({ ros, name: (ROS_CONFIG.nav?.statusTopic || '/navigate_to_pose/status'), messageType: ROS_CONFIG.messageTypes.navStatus, throttle_rate: 1000, queue_length: 1 }),  // 1 Hz
};

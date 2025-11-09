import ROSLIB from 'roslib';
import { ROS_CONFIG } from './config';

export const ros = new ROSLIB.Ros({
  url: ROS_CONFIG.rosbridgeUrl
});

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
export const topics = {
  // Core
  cmdVel: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.cmdVel, messageType: ROS_CONFIG.messageTypes.cmdVel }),
  odom: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.odom, messageType: ROS_CONFIG.messageTypes.odom }),
  battery: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.battery, messageType: ROS_CONFIG.messageTypes.battery }),

  // Optional UI bridge topics (subscribe only if available in your system)
  robotState: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.robotState, messageType: ROS_CONFIG.messageTypes.robotState }),
  robotLog: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.robotLog, messageType: ROS_CONFIG.messageTypes.robotLog }),
  cmdFeedback: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.cmdFeedback, messageType: ROS_CONFIG.messageTypes.cmdFeedback }),
  uiCmd: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.uiCmd, messageType: ROS_CONFIG.messageTypes.uiCmd }),
  uiCancel: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.uiCancel, messageType: ROS_CONFIG.messageTypes.uiCancel }),
  rosout: new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.rosout, messageType: ROS_CONFIG.messageTypes.rosout }),
};

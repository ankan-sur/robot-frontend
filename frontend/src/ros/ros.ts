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


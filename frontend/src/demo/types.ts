/**
 * Demo Control Client - Types and Interface
 * 
 * Abstraction layer for robot control. This allows the UI to work with
 * either a mock implementation (for development) or a real ROS/backend
 * implementation (for Demo Day).
 */

import { POI } from './pois';

// Re-export POI type
export type { POI } from './pois';

// Navigation state machine
export type DemoNavState = 
  | 'idle'       // Robot is idle, ready for commands
  | 'sending'    // Command being sent to robot
  | 'navigating' // Robot is moving to destination
  | 'arrived'    // Robot reached destination
  | 'cancelled'  // Navigation was cancelled
  | 'failed';    // Navigation failed

// Robot status snapshot
export interface DemoRobotStatus {
  connected: boolean;
  batteryPercent?: number;
  mode?: 'idle' | 'slam' | 'nav';
  currentDestination?: POI | null;
  navState: DemoNavState;
  navMessage?: string;
  // Robot pose (for displaying on map)
  pose?: {
    x: number;
    y: number;
    theta: number;
  };
}

// Status change callback
export type StatusCallback = (status: DemoRobotStatus) => void;

// Control client interface
export interface DemoControlClient {
  /**
   * Subscribe to robot status updates
   * @param callback Called whenever status changes
   * @returns Unsubscribe function
   */
  onStatusUpdate(callback: StatusCallback): () => void;
  
  /**
   * Get current status synchronously
   */
  getStatus(): DemoRobotStatus;
  
  /**
   * Send robot to a POI
   * @param poi The POI to navigate to
   * @returns Promise that resolves when navigation completes or rejects on failure
   */
  navigateTo(poi: POI): Promise<void>;
  
  /**
   * Cancel current navigation
   */
  cancelNavigation(): Promise<void>;
  
  /**
   * Connect to the robot/backend
   */
  connect(): Promise<void>;
  
  /**
   * Disconnect and cleanup
   */
  disconnect(): void;
}

// Initial status (disconnected)
export const INITIAL_STATUS: DemoRobotStatus = {
  connected: false,
  navState: 'idle',
  mode: 'nav',
  currentDestination: null,
};

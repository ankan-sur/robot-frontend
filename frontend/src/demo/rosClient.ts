/**
 * Demo Control Client - ROS Implementation (Stub)
 * 
 * Real implementation that connects to rosbridge and sends NavigateToPose
 * actions to Nav2. This is a stub with TODOs - implement before Demo Day.
 */

import { 
  DemoControlClient, 
  DemoRobotStatus, 
  StatusCallback,
  INITIAL_STATUS,
  POI,
} from './types';
import { DEMO_CONFIG } from './config';

// TODO(Anka): Import ROSLIB when ready to implement
// import ROSLIB from 'roslib';

export class RosDemoControlClient implements DemoControlClient {
  private status: DemoRobotStatus = { ...INITIAL_STATUS };
  private subscribers: Set<StatusCallback> = new Set();
  // private ros: ROSLIB.Ros | null = null;
  // private actionClient: ROSLIB.ActionClient | null = null;
  // private currentGoal: ROSLIB.Goal | null = null;

  constructor() {
    console.log('[RosDemoClient] Initialized');
  }

  onStatusUpdate(callback: StatusCallback): () => void {
    this.subscribers.add(callback);
    callback(this.status);
    
    return () => {
      this.subscribers.delete(callback);
    };
  }

  getStatus(): DemoRobotStatus {
    return { ...this.status };
  }

  private updateStatus(partial: Partial<DemoRobotStatus>): void {
    this.status = { ...this.status, ...partial };
    this.subscribers.forEach(cb => cb({ ...this.status }));
  }

  async connect(): Promise<void> {
    console.log(`[RosDemoClient] Connecting to ${DEMO_CONFIG.backendUrl}...`);
    
    // TODO(Anka): Implement rosbridge connection
    // Example:
    // this.ros = new ROSLIB.Ros({ url: DEMO_CONFIG.backendUrl });
    // 
    // return new Promise((resolve, reject) => {
    //   this.ros.on('connection', () => {
    //     console.log('[RosDemoClient] Connected to rosbridge');
    //     this.setupSubscriptions();
    //     this.updateStatus({ connected: true, mode: 'nav', navState: 'idle' });
    //     resolve();
    //   });
    //   this.ros.on('error', reject);
    // });

    throw new Error('ROS client not implemented yet - use mock client');
  }

  disconnect(): void {
    console.log('[RosDemoClient] Disconnecting...');
    
    // TODO(Anka): Implement cleanup
    // if (this.currentGoal) {
    //   this.currentGoal.cancel();
    // }
    // if (this.ros) {
    //   this.ros.close();
    // }
    
    this.updateStatus({ connected: false, navState: 'idle', currentDestination: null });
  }

  async navigateTo(poi: POI): Promise<void> {
    console.log(`[RosDemoClient] Navigating to ${poi.label} at (${poi.x}, ${poi.y}, ${poi.theta})`);
    
    // TODO(Anka): Implement NavigateToPose action call
    // Example:
    // this.actionClient = new ROSLIB.ActionClient({
    //   ros: this.ros,
    //   serverName: '/navigate_to_pose',
    //   actionName: 'nav2_msgs/action/NavigateToPose'
    // });
    //
    // const goal = new ROSLIB.Goal({
    //   actionClient: this.actionClient,
    //   goalMessage: {
    //     pose: {
    //       header: { frame_id: 'map' },
    //       pose: {
    //         position: { x: poi.x, y: poi.y, z: 0 },
    //         orientation: quaternionFromYaw(poi.theta)
    //       }
    //     }
    //   }
    // });
    //
    // goal.on('feedback', (feedback) => {
    //   this.updateStatus({
    //     navState: 'navigating',
    //     navMessage: `Distance remaining: ${feedback.distance_remaining.toFixed(1)}m`
    //   });
    // });
    //
    // goal.on('result', (result) => {
    //   if (result.status === 4) { // SUCCEEDED
    //     this.updateStatus({ navState: 'arrived', navMessage: `Arrived at ${poi.label}` });
    //   } else {
    //     this.updateStatus({ navState: 'failed', navMessage: 'Navigation failed' });
    //   }
    // });
    //
    // goal.send();
    // this.currentGoal = goal;

    throw new Error('ROS client not implemented yet - use mock client');
  }

  async cancelNavigation(): Promise<void> {
    console.log('[RosDemoClient] Cancelling navigation');
    
    // TODO(Anka): Implement goal cancellation
    // if (this.currentGoal) {
    //   this.currentGoal.cancel();
    //   this.currentGoal = null;
    // }
    
    this.updateStatus({
      navState: 'cancelled',
      navMessage: 'Navigation cancelled',
      currentDestination: null,
    });
  }

  // TODO(Anka): Implement these helpers when connecting to ROS
  // private setupSubscriptions(): void {
  //   // Subscribe to battery
  //   const batteryTopic = new ROSLIB.Topic({
  //     ros: this.ros,
  //     name: '/ros_robot_controller/battery',
  //     messageType: 'std_msgs/UInt16'
  //   });
  //   batteryTopic.subscribe((msg) => {
  //     const percent = ((msg.data - 6000) / 2400) * 100;
  //     this.updateStatus({ batteryPercent: Math.round(percent) });
  //   });
  //
  //   // Subscribe to robot pose (from AMCL or odometry)
  //   const poseTopic = new ROSLIB.Topic({
  //     ros: this.ros,
  //     name: '/amcl_pose',
  //     messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  //   });
  //   poseTopic.subscribe((msg) => {
  //     const pos = msg.pose.pose.position;
  //     const q = msg.pose.pose.orientation;
  //     const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  //     this.updateStatus({ pose: { x: pos.x, y: pos.y, theta } });
  //   });
  // }
}

// Helper to convert yaw to quaternion
// function quaternionFromYaw(yaw: number) {
//   return {
//     x: 0,
//     y: 0,
//     z: Math.sin(yaw / 2),
//     w: Math.cos(yaw / 2)
//   };
// }

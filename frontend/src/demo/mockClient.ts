/**
 * Demo Control Client - Mock Implementation
 * 
 * Fake implementation for UI development without a live robot.
 * Simulates navigation state transitions with configurable timing.
 */

import { 
  DemoControlClient, 
  DemoRobotStatus, 
  StatusCallback,
  INITIAL_STATUS,
  POI,
} from './types';
import { DEMO_CONFIG } from './config';

export class MockDemoControlClient implements DemoControlClient {
  private status: DemoRobotStatus = { ...INITIAL_STATUS };
  private subscribers: Set<StatusCallback> = new Set();
  private navigationTimer?: ReturnType<typeof setTimeout>;
  private connected: boolean = false;

  constructor() {
    console.log('[MockDemoClient] Initialized - using simulated robot');
  }

  onStatusUpdate(callback: StatusCallback): () => void {
    this.subscribers.add(callback);
    // Immediately send current status
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
    this.notifySubscribers();
  }

  private notifySubscribers(): void {
    const statusCopy = { ...this.status };
    this.subscribers.forEach(cb => cb(statusCopy));
  }

  async connect(): Promise<void> {
    console.log('[MockDemoClient] Connecting...');
    
    // Simulate connection delay
    await new Promise(resolve => setTimeout(resolve, 500));
    
    this.connected = true;
    this.updateStatus({
      connected: true,
      batteryPercent: 85, // Simulated battery
      mode: 'nav',
      navState: 'idle',
      navMessage: 'Ready for commands',
      currentDestination: null,
      // Start robot at dock position
      pose: { x: 0, y: 0, theta: 0 },
    });
    
    console.log('[MockDemoClient] Connected (simulated)');
  }

  disconnect(): void {
    console.log('[MockDemoClient] Disconnecting...');
    
    if (this.navigationTimer) {
      clearTimeout(this.navigationTimer);
      this.navigationTimer = undefined;
    }
    
    this.connected = false;
    this.updateStatus({
      connected: false,
      navState: 'idle',
      navMessage: 'Disconnected',
      currentDestination: null,
    });
  }

  async navigateTo(poi: POI): Promise<void> {
    if (!this.connected) {
      throw new Error('Not connected to robot');
    }

    console.log(`[MockDemoClient] Sending robot to ${poi.label} (${poi.id})`);
    
    // Cancel any existing navigation
    if (this.navigationTimer) {
      clearTimeout(this.navigationTimer);
    }

    // Phase 1: Sending
    this.updateStatus({
      navState: 'sending',
      currentDestination: poi,
      navMessage: `Sending command...`,
    });

    // Simulate command send delay
    await new Promise(resolve => 
      setTimeout(resolve, DEMO_CONFIG.mockSendingDelayMs)
    );

    // Phase 2: Navigating
    this.updateStatus({
      navState: 'navigating',
      navMessage: `Navigating to ${poi.label}...`,
    });

    // Phase 3: Simulate navigation progress and arrival
    return new Promise((resolve) => {
      const startPose = this.status.pose || { x: 0, y: 0, theta: 0 };
      const targetPose = { x: poi.x, y: poi.y, theta: poi.theta };
      const duration = DEMO_CONFIG.mockNavigationDurationMs;
      const startTime = Date.now();

      const updateProgress = () => {
        const elapsed = Date.now() - startTime;
        const progress = Math.min(elapsed / duration, 1);

        // Interpolate robot position
        const currentPose = {
          x: startPose.x + (targetPose.x - startPose.x) * progress,
          y: startPose.y + (targetPose.y - startPose.y) * progress,
          theta: startPose.theta + (targetPose.theta - startPose.theta) * progress,
        };

        if (progress < 1) {
          this.updateStatus({
            pose: currentPose,
            navMessage: `Navigating to ${poi.label}... ${Math.round(progress * 100)}%`,
          });
          this.navigationTimer = setTimeout(updateProgress, 200);
        } else {
          // Arrived!
          this.updateStatus({
            navState: 'arrived',
            pose: targetPose,
            navMessage: `Arrived at ${poi.label}`,
          });
          
          console.log(`[MockDemoClient] Arrived at ${poi.label}`);
          
          // After a moment, go back to idle
          this.navigationTimer = setTimeout(() => {
            this.updateStatus({
              navState: 'idle',
              currentDestination: null,
              navMessage: 'Ready for commands',
            });
          }, 3000);
          
          resolve();
        }
      };

      updateProgress();
    });
  }

  async cancelNavigation(): Promise<void> {
    console.log('[MockDemoClient] Cancelling navigation');
    
    if (this.navigationTimer) {
      clearTimeout(this.navigationTimer);
      this.navigationTimer = undefined;
    }

    const goalName = this.status.currentDestination?.label ?? 'destination';

    this.updateStatus({
      navState: 'cancelled',
      navMessage: `Navigation to ${goalName} cancelled`,
    });

    // After a moment, go back to idle
    setTimeout(() => {
      this.updateStatus({
        navState: 'idle',
        currentDestination: null,
        navMessage: 'Ready for commands',
      });
    }, 2000);
  }
}

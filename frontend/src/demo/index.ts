/**
 * Demo Control Client Factory
 * 
 * Creates the appropriate client based on configuration.
 */

import { DemoControlClient } from './types';
import { MockDemoControlClient } from './mockClient';
import { RosDemoControlClient } from './rosClient';
import { DEMO_CONFIG } from './config';

let clientInstance: DemoControlClient | null = null;

/**
 * Get or create the demo control client singleton
 */
export function getDemoClient(): DemoControlClient {
  if (!clientInstance) {
    if (DEMO_CONFIG.useMockClient) {
      console.log('[Demo] Using mock client for development');
      clientInstance = new MockDemoControlClient();
    } else {
      console.log('[Demo] Using ROS client');
      clientInstance = new RosDemoControlClient();
    }
  }
  return clientInstance;
}

/**
 * Reset the client (for testing or reconnection)
 */
export function resetDemoClient(): void {
  if (clientInstance) {
    clientInstance.disconnect();
    clientInstance = null;
  }
}

// Re-export types and POIs for convenience
export * from './types';
export * from './pois';
export { DEMO_CONFIG } from './config';

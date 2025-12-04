/**
 * Demo UI Configuration
 * 
 * Configuration for the Demo Day UI.
 */

export const DEMO_CONFIG = {
  // Robot configuration
  robotName: 'HFH Robot',
  robotId: 'hfhrobot',
  
  // Map configuration - demohall1
  mapName: 'demohall1',
  
  // Map display settings (pixels)
  mapWidth: 800,
  mapHeight: 400,
  
  // Map origin in ROS coordinates (from map.yaml)
  // TODO: Get these from demohall1.yaml file
  mapOrigin: { x: -10.0, y: -5.0 },
  mapResolution: 0.05, // meters per pixel
  
  // Rosbridge URL - LAN only
  rosbridgeUrl: import.meta.env.VITE_ROSBRIDGE_URL || 'ws://192.168.149.1:9090',
  
  // Backend URL (unused in demo - LAN only)
  backendUrl: 'ws://localhost:9090',
  
  // Use mock/dummy implementation (set false for real robot)
  useMockClient: import.meta.env.VITE_DEMO_USE_MOCK === 'true',
  
  // Simulated navigation timing (for mock client)
  mockNavigationDurationMs: 8000,
  mockSendingDelayMs: 500,
  
  // UI refresh rate
  statusPollIntervalMs: 1000,
};

export type DemoConfig = typeof DEMO_CONFIG;

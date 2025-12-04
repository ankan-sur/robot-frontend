/**
 * Demo UI Configuration
 * 
 * Configuration for the Demo Day UI. All values here are placeholders
 * that will be replaced with real data before Demo Day.
 */

// TODO(Anka): Replace with real map + POI coordinates from RViz before Demo Day.

export const DEMO_CONFIG = {
  // Robot configuration
  robotName: 'MentorPi',
  robotId: 'mentorpi',
  
  // Map configuration
  // TODO(Anka): Replace with actual map name from slam/maps/
  mapName: 'hallway_demo',
  
  // Map display settings (pixels)
  mapWidth: 800,
  mapHeight: 400,
  
  // Map origin in ROS coordinates (from map.yaml)
  // TODO(Anka): Get these from your hallway map's .yaml file
  mapOrigin: { x: -10.0, y: -5.0 },
  mapResolution: 0.05, // meters per pixel
  
  // Backend URL (if using HTTP backend instead of rosbridge)
  // TODO(Anka): Set this if you decide to use a demo backend
  backendUrl: import.meta.env.VITE_DEMO_BACKEND_URL || 'ws://localhost:9090',
  
  // Use mock/dummy implementation (set false when connecting to real robot)
  useMockClient: import.meta.env.VITE_DEMO_USE_MOCK !== 'false',
  
  // Simulated navigation timing (for mock client)
  mockNavigationDurationMs: 8000,
  mockSendingDelayMs: 500,
  
  // UI refresh rate
  statusPollIntervalMs: 1000,
};

export type DemoConfig = typeof DEMO_CONFIG;

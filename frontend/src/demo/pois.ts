/**
 * Demo POI Definitions for demohall1 map
 * 
 * Points of Interest for the Demo Day UI.
 * 
 * TODO: Replace x, y, theta with real RViz measurements.
 * To get coordinates:
 * 1. Open RViz with demohall1 map loaded
 * 2. Use "Publish Point" tool to click on each location
 * 3. Read the x, y coordinates from the terminal
 * 4. For theta (orientation), measure the angle the robot should face (radians)
 *    - 0 = facing positive X (right)
 *    - π/2 = facing positive Y (up)
 *    - π = facing negative X (left)
 *    - -π/2 = facing negative Y (down)
 */

export interface POI {
  id: string;
  label: string;       // Display name
  icon: string;        // Emoji icon
  description?: string;
  x: number;           // meters in map frame
  y: number;           // meters in map frame
  theta: number;       // radians, robot heading at destination
  // Display position on the map image (percentage from top-left)
  displayX: number;    // 0-100%
  displayY: number;    // 0-100%
  color?: string;      // Optional custom color
}

// Alias for compatibility
export type DemoPoi = POI;

// POIs for demohall1 - hallway map
// TODO: Update these with actual coordinates from RViz after loading demohall1
export const DEMO_POIS: POI[] = [
  {
    id: 'start',
    label: 'Start Position',
    icon: '🏠',
    description: 'Starting/home position',
    x: 0.0,
    y: 0.0,
    theta: 0,
    displayX: 10,
    displayY: 50,
    color: '#10b981', // emerald
  },
  {
    id: 'point1',
    label: 'Point A',
    icon: '�',
    description: 'First waypoint',
    x: 2.0,
    y: 0.5,
    theta: 0,
    displayX: 30,
    displayY: 40,
    color: '#3b82f6', // blue
  },
  {
    id: 'point2',
    label: 'Point B',
    icon: '📍',
    description: 'Second waypoint',
    x: 4.0,
    y: 0.0,
    theta: 0,
    displayX: 50,
    displayY: 50,
    color: '#8b5cf6', // violet
  },
  {
    id: 'end',
    label: 'End Point',
    icon: '🎯',
    description: 'End of hallway',
    x: 6.0,
    y: -0.5,
    theta: Math.PI, // facing back
    displayX: 70,
    displayY: 60,
    color: '#f59e0b', // amber
  },
];

// Helper to find POI by ID
export function getPoiById(id: string): POI | undefined {
  return DEMO_POIS.find(poi => poi.id === id);
}

// Helper to get POI display name
export function getPoiName(id: string): string {
  return getPoiById(id)?.label ?? id;
}

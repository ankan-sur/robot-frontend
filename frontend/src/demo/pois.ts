/**
 * Demo POI Definitions
 * 
 * Points of Interest for the Demo Day UI.
 * These are placeholder coordinates - replace with real RViz measurements.
 */

// TODO(Anka): Replace with real map + POI coordinates from RViz before Demo Day.
// To get coordinates:
// 1. Open RViz with your hallway map loaded
// 2. Use "Publish Point" tool to click on each location
// 3. Read the x, y coordinates from the terminal
// 4. For theta (orientation), measure the angle the robot should face (radians)
//    - 0 = facing positive X (right)
//    - π/2 = facing positive Y (up)
//    - π = facing negative X (left)
//    - -π/2 = facing negative Y (down)

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

export const DEMO_POIS: POI[] = [
  {
    id: 'doorA',
    label: 'Door A',
    icon: '🚪',
    description: 'First door on the left',
    // TODO(Anka): Replace with real coordinates from RViz
    x: 2.5,
    y: 1.0,
    theta: Math.PI / 2, // Facing the door (up)
    // Display position on map image
    displayX: 25,
    displayY: 30,
    color: '#10b981', // emerald-500
  },
  {
    id: 'doorB',
    label: 'Door B',
    icon: '🚪',
    description: 'Second door on the right',
    // TODO(Anka): Replace with real coordinates from RViz
    x: 5.0,
    y: -1.0,
    theta: -Math.PI / 2, // Facing the door (down)
    // Display position on map image
    displayX: 55,
    displayY: 70,
    color: '#06b6d4', // cyan-500
  },
  {
    id: 'dock',
    label: 'Docking Station',
    icon: '🔌',
    description: 'Robot home/charging station',
    // TODO(Anka): Replace with real coordinates from RViz
    x: 0.0,
    y: 0.0,
    theta: 0, // Facing forward (right)
    // Display position on map image
    displayX: 85,
    displayY: 50,
    color: '#f59e0b', // amber-500
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

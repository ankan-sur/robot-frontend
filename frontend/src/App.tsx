import { useState } from 'react'
import { Header } from './components/Header'
import { Controls } from './components/Controls'
import VideoFeed from './components/VideoFeed'
import { MapView } from './components/MapView'
import { TelemetryPanel } from './components/TelemetryPanel'
import { DebugPanel } from './components/DebugPanel'
import { useRosConnection, useOdom, useCmdVel, useNavigateToPose, PointOfInterest } from './ros/hooks'

export default function App() {
  const { connected } = useRosConnection();
  const odom = useOdom();
  const { stop } = useCmdVel();
  const { navigate } = useNavigateToPose();
  const [showDebug, setShowDebug] = useState(false);

  // Extract position from odometry
  const position = odom?.pose.pose.position || { x: 0, y: 0, z: 0 };

  // Navigate to POI using Nav2
  const goToLab = async (poi: PointOfInterest) => {
    if (!connected) {
      alert('Not connected to ROS');
      return;
    }
    
    try {
      await navigate(poi.x, poi.y, poi.yaw || 0);
      console.log(`Navigated to ${poi.name}`);
    } catch (error: any) {
      console.error('Navigation error:', error);
      alert(`Navigation failed: ${error.message}`);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-blue-100">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-4 grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <MapView position={position} />
          <VideoFeed />
          {showDebug && <DebugPanel />}
        </div>
        <div className="lg:col-span-1 space-y-4">
          <Controls
            goToLab={goToLab}
            onStop={stop}
            disabledMove={!connected}
            disabledStop={!connected}
            controlAllowed={connected}
          />
          <TelemetryPanel />
          <div className="text-center">
            <button
              onClick={() => setShowDebug(!showDebug)}
              className="text-xs text-blue-600 hover:text-blue-800 underline"
            >
              {showDebug ? 'Hide' : 'Show'} Debug Panel
            </button>
          </div>
        </div>
      </main>
    </div>
  )
}


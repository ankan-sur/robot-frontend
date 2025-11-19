import { Header } from './components/Header'
import { Controls } from './components/Controls'
import { MapView } from './components/MapView'
import { TelemetryPanel } from './components/TelemetryPanel'
import { DebugPanel } from './components/DebugPanel'
import { MappingPanel } from './components/MappingPanel'
import VideoFeed from './components/VideoFeed'
import { useRosConnection, useCmdVel, useNavigateToPose, PointOfInterest } from './ros/hooks'

export default function App() {
  const { connected } = useRosConnection();
  const { stop } = useCmdVel();
  const { navigate } = useNavigateToPose();

  // Navigate to POI using Nav2
  const goToLab = async (poi: PointOfInterest) => {
    if (!connected) {
      alert('Not connected to ROS');
      return;
    }

    try {
      const x = poi.pose?.x ?? poi.x ?? 0;
      const y = poi.pose?.y ?? poi.y ?? 0;
      const yaw = poi.pose?.yaw ?? poi.yaw ?? 0;
      await navigate(x, y, yaw);
      console.log(`Navigated to ${poi.name}`);
    } catch (error: any) {
      console.error('Navigation error:', error);
      alert(`Navigation failed: ${error.message}`);
    }
  };

  return (
    <div className="min-h-screen bg-slate-50">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-3 space-y-3">
        {/* Top: Camera and Telemetry side-by-side on desktop, stacked on mobile */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
          <VideoFeed />
          <TelemetryPanel />
        </div>
        
        {/* Map View */}
        <MapView />
        
        {/* Consolidated Control Panel (Mapping + Mode + Navigation) */}
        <MappingPanel goToLab={goToLab} onStop={stop} disabledMove={!connected} />
        
        {/* Debug panel at bottom */}
        <DebugPanel />
      </main>
    </div>
  )
}

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
      await navigate(poi.x, poi.y, poi.yaw || 0);
      console.log(`Navigated to ${poi.name}`);
    } catch (error: any) {
      console.error('Navigation error:', error);
      alert(`Navigation failed: ${error.message}`);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 via-slate-50 to-slate-100">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-4 grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <MapView />
          <DebugPanel />
        </div>
        <div className="lg:col-span-1 space-y-4">
          <VideoFeed />
          <MappingPanel />
          <Controls
            goToLab={goToLab}
            onStop={stop}
            disabledMove={!connected}
          />
          <TelemetryPanel />
        </div>
      </main>
    </div>
  )
}

import { Header } from './components/Header'
import { MapView } from './components/MapView'
import { TelemetryPanel } from './components/TelemetryPanel'
import { DebugPanel } from './components/DebugPanel'
import { MappingPanel } from './components/MappingPanel'
import MapCameraTabs from './components/MapCameraTabs'
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
        {/* Main working area: Map + Controls on the left, Telemetry + Debug sticky on the right */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-3">
          {/* Left: Map and Control Center stacked */}
          <div className="lg:col-span-2 flex flex-col gap-3">
            <MapCameraTabs />
            {/* Consolidated Control Panel (Mapping + Mode + Navigation) */}
            <MappingPanel goToLab={goToLab} onStop={stop} disabledMove={!connected} />
          </div>

          {/* Right: Telemetry + Debug kept visible for easier debugging */}
          <div className="space-y-3 lg:sticky lg:top-3 self-start">
            <TelemetryPanel />
            <DebugPanel />
          </div>
        </div>

        {/* Extra space saved by moving camera into tabs above */}
      </main>
    </div>
  )
}

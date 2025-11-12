import { Header } from './components/Header'
import { Controls } from './components/Controls'
import VideoFeed from './components/VideoFeed'
import { MapView } from './components/MapView'
import { TelemetryPanel } from './components/TelemetryPanel'
import { DebugPanel } from './components/DebugPanel'
import { DebugLog } from './components/DebugLog'
import { useRosConnection, useCmdVel } from './ros/hooks'

export default function App() {
  const { connected } = useRosConnection();
  const { stop } = useCmdVel();
  // Navigation to POIs removed from UI per request

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 via-slate-50 to-slate-100">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-4 grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <MapView />
          <VideoFeed />
          <DebugLog />
          <DebugPanel />
        </div>
        <div className="lg:col-span-1 space-y-4">
          <Controls onStop={stop} disabledMove={!connected} />
          <TelemetryPanel />
        </div>
      </main>
    </div>
  )
}

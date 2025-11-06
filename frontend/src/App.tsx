import { useEffect, useRef, useState } from 'react'
import { Header } from './components/Header'
import { Controls } from './components/Controls'
import VideoFeed from './components/VideoFeed'
import { MapView } from './components/MapView'
import { useRosConnection, useOdom, useBattery, useCmdVel } from './ros/hooks'

export default function App() {
  const { connected, state, latency } = useRosConnection();
  const odom = useOdom();
  const battery = useBattery();
  const { send: sendCmdVel, stop } = useCmdVel();

  // Robot log
  const [logs, setLogs] = useState<string[]>([])
  const lastOdomRef = useRef<any>(null);
  const lastBatteryRef = useRef<number | null>(null);

  useEffect(() => {
    const ts = new Date().toLocaleTimeString();
    
    if (odom && odom !== lastOdomRef.current) {
      const { x, y } = odom.pose.pose.position;
      lastOdomRef.current = odom;
      setLogs(prev => [`${ts} • pos: (${x.toFixed(2)}, ${y.toFixed(2)})`, ...prev].slice(0, 100));
    }
    
    if (battery !== null && battery !== lastBatteryRef.current) {
      lastBatteryRef.current = battery;
      setLogs(prev => [`${ts} • battery: ${battery.toFixed(1)}%`, ...prev].slice(0, 100));
    }
  }, [odom, battery]);

  // Go to Lab - send navigation commands
  const goToLab = async (labIdx: number) => {
    if (!connected) return;
    
    // Simple movement: adjust based on your navigation needs
    // For now, just send a forward command
    const speed = 0.3;
    sendCmdVel(speed, 0); // Move forward
    
    // In a real implementation, you'd use Nav2 action client
    // For now, this is a placeholder
  };

  // Extract position from odometry
  const position = odom?.pose.pose.position || { x: 0, y: 0, z: 0 };
  const batteryLevel = battery ?? 0;

  return (
    <div className="min-h-screen bg-slate-50 text-slate-900">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-4 grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <MapView position={position} />
          <VideoFeed />
        </div>
        <div className="lg:col-span-1 space-y-4">
          <Controls
            goToLab={goToLab}
            onStop={stop}
            disabledMove={!connected}
            disabledStop={!connected}
            controlAllowed={connected}
          />
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Battery</h2>
            <div className="bg-slate-50 rounded p-3 text-sm">
              <div className="text-slate-500">Level</div>
              <div className="text-slate-800">{batteryLevel.toFixed(1)}%</div>
            </div>
          </section>
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Connection Status</h2>
            <div className="bg-slate-50 rounded p-3 text-sm space-y-2">
              <div>
                <div className="text-slate-500">ROS Bridge</div>
                <div className={`flex items-center gap-2 ${connected ? 'text-emerald-600' : 'text-rose-600'}`}>
                  <span className={`w-2 h-2 rounded-full ${connected ? 'bg-emerald-600' : 'bg-rose-600'}`}></span>
                  <span>{state}</span>
                </div>
              </div>
              {latency !== null && (
                <div>
                  <div className="text-slate-500">Latency</div>
                  <div className="text-slate-800">{latency}ms</div>
                </div>
              )}
            </div>
          </section>
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Position</h2>
            <div className="bg-slate-50 rounded p-3 text-sm">
              <div className="text-slate-500">X: {position.x.toFixed(2)}</div>
              <div className="text-slate-500">Y: {position.y.toFixed(2)}</div>
            </div>
          </section>
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Robot Log</h2>
            <div className="h-40 overflow-auto rounded border border-slate-200 bg-slate-50 p-2 text-xs text-slate-700">
              {logs.length === 0 ? (
                <div className="text-slate-400">No logs yet…</div>
              ) : (
                <ul className="space-y-1">
                  {logs.map((line, idx) => (
                    <li key={idx} className="font-mono">{line}</li>
                  ))}
                </ul>
              )}
            </div>
          </section>
        </div>
      </main>
    </div>
  )
}


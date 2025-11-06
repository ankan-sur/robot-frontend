import { useEffect, useMemo, useRef, useState } from 'react'
import { Header } from './components/Header'
import { Controls } from './components/Controls'
import VideoFeed from './components/VideoFeed'
import { MapView } from './components/MapView'
import { useTelemetry } from './hooks/useTelemetry'
import { apiMove, apiStop } from './lib/api'

export default function App() {
  const apiBase = ((import.meta as any).env.VITE_API_BASE) ?? 'http://localhost:8000'
  const wsUrl = useMemo(() => {
    const httpBase = apiBase.replace('https://', 'wss://').replace('http://', 'ws://')
    return `${httpBase}/ws/telemetry`
  }, [apiBase])

  const { telemetry, ok } = useTelemetry(wsUrl, `${apiBase}/status`)
  const token = (import.meta as any).env.VITE_CONTROL_TOKEN as string | undefined

  const controlAllowed = ok && (telemetry?.sdk_available ?? true) && Boolean(telemetry?.control_allowed)
  const battery = telemetry?.battery ?? 0

  // Robot log (simple derived log from telemetry changes)
  const [logs, setLogs] = useState<string[]>([])
  const lastCommandRef = useRef<string | undefined>(undefined)
  useEffect(() => {
    const ts = new Date().toLocaleTimeString()
    if (telemetry?.last_command && telemetry.last_command !== lastCommandRef.current) {
      lastCommandRef.current = telemetry.last_command
      setLogs(prev => [`${ts} • cmd: ${telemetry.last_command}`, ...prev].slice(0, 100))
    }
    if (typeof telemetry?.battery === 'number') {
      setLogs(prev => [`${ts} • battery: ${telemetry.battery.toFixed(1)}%`, ...prev].slice(0, 100))
    }
  }, [telemetry])

  // Go to Lab buttons
  const goToLab = async (labIdx: number) => {
    if (!controlAllowed) return
    // This assumes direction/command for the backend – update as needed
    const label = labIdx === 1 ? 'lab1' : 'lab2'
    await apiMove(apiBase, label, undefined, token)
  }

  useEffect(() => {
    // Adjust defaults if needed
  }, [])

  return (
    <div className="min-h-screen bg-slate-50 text-slate-900">
      <Header title="HFH Robot Dashboard" />
      <main className="max-w-7xl mx-auto p-4 grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2 space-y-4">
          <MapView />
          <VideoFeed streamUrl={apiBase + '/video'} />
        </div>
        <div className="lg:col-span-1 space-y-4">
          <Controls
            goToLab={goToLab}
            onStop={async () => { await apiStop(apiBase, token) }}
            disabledMove={!controlAllowed}
            disabledStop={false}
            controlAllowed={controlAllowed}
          />
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Battery</h2>
            <div className="bg-slate-50 rounded p-3 text-sm">
              <div className="text-slate-500">Level</div>
              <div className="text-slate-800">{battery.toFixed(1)}%</div>
            </div>
          </section>
          <section className="rounded-lg border border-slate-200 bg-white p-4">
            <h2 className="text-lg font-medium mb-3 text-slate-800">Connection Status</h2>
            <div className="bg-slate-50 rounded p-3 text-sm">
              <div className="text-slate-500">Robot</div>
              <div className={`flex items-center gap-2 ${ok ? 'text-emerald-600' : 'text-rose-600'}`}>
                <span className={`w-2 h-2 rounded-full ${ok ? 'bg-emerald-600' : 'bg-rose-600'}`}></span>
                <span>{ok ? 'Connected' : 'Disconnected'}</span>
              </div>
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


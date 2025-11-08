type Telemetry = {
  battery?: number
  is_moving?: boolean
  last_command?: string
  odom?: { x?: number; y?: number; theta?: number }
  control_allowed?: boolean
  wifi_strength?: number
  ip_address?: string
  cpu_temp?: number
  system_uptime?: number
  pose_map?: { x: number; y: number }
}

type Props = {
  telemetry?: Telemetry
  ok: boolean
}

export function Dashboard({ telemetry, ok }: Props) {
  const battery = telemetry?.battery ?? 0
  const isMoving = telemetry?.is_moving ?? false
  const cmd = telemetry?.last_command ?? '—'
  const { x = 0, y = 0, theta = 0 } = telemetry?.odom ?? {}
  const allowed = telemetry?.control_allowed ?? false

  return (
    <section className="rounded-lg border border-slate-200 bg-white p-4">
      <h2 className="text-lg font-medium mb-3 text-slate-800">Telemetry</h2>
      <div className="grid grid-cols-2 md:grid-cols-3 gap-4 text-sm">
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">Backend</div>
          <div className={ok ? 'text-emerald-600' : 'text-rose-600'}>{ok ? 'Connected' : 'Offline'}</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">Battery</div>
          <div className="text-slate-800">{battery?.toFixed(1) ?? 'N/A'}%</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">WiFi Signal</div>
          <div className="text-slate-800">{telemetry?.wifi_strength ? `${telemetry.wifi_strength}%` : 'N/A'}</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">IP Address</div>
          <div className="text-slate-800 font-mono text-xs">{telemetry?.ip_address || 'N/A'}</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">CPU Temp</div>
          <div className="text-slate-800">{telemetry?.cpu_temp ? `${telemetry.cpu_temp.toFixed(1)}°C` : 'N/A'}</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">System Uptime</div>
          <div className="text-slate-800">{telemetry?.system_uptime ? `${Math.floor(telemetry.system_uptime / 3600)}h ${Math.floor((telemetry.system_uptime % 3600) / 60)}m` : 'N/A'}</div>
        </div>
      </div>
      <div className="mt-4 space-y-2 text-xs">
        <div className="text-slate-500">
          Last command: <span className="text-slate-700">{cmd}</span> | 
          Moving: <span className="text-slate-700">{isMoving ? 'Yes' : 'No'}</span>
        </div>
        {telemetry?.pose_map && (
          <div className="text-slate-500">
            Map position: <span className="text-slate-700 font-mono">({telemetry.pose_map.x.toFixed(2)}, {telemetry.pose_map.y.toFixed(2)})</span>
          </div>
        )}
      </div>
    </section>
  )
}


type Telemetry = {
  battery?: number
  is_moving?: boolean
  last_command?: string
  odom?: { x?: number; y?: number; theta?: number }
  control_allowed?: boolean
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
      <div className="grid grid-cols-2 gap-4 text-sm">
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">Backend</div>
          <div className={ok ? 'text-emerald-600' : 'text-rose-600'}>{ok ? 'Connected' : 'Offline'}</div>
        </div>
        <div className="bg-slate-50 rounded p-3">
          <div className="text-slate-500">Battery</div>
          <div className="text-slate-800">{battery.toFixed(1)}%</div>
        </div>
      </div>
      <div className="mt-4 text-xs text-slate-500">Last command: {cmd} | Moving: {isMoving ? 'Yes' : 'No'}</div>
    </section>
  )
}


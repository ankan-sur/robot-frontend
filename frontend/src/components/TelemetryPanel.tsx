import { useRosConnection, useBattery, useOdom } from '../ros/hooks'

export function TelemetryPanel() {
  const { connected, state, latency } = useRosConnection()
  const battery = useBattery()
  const odom = useOdom()

  const position = odom?.pose.pose.position || { x: 0, y: 0, z: 0 }
  const velocity = odom?.twist.linear || { x: 0, y: 0, z: 0 }
  const angularVel = odom?.twist.angular || { x: 0, y: 0, z: 0 }
  const speed = Math.sqrt(velocity.x ** 2 + velocity.y ** 2)

  return (
    <section className="rounded-lg border-2 border-blue-300 bg-white p-4 shadow-md">
      <h2 className="text-lg font-semibold mb-3 text-blue-900">Telemetry</h2>
      <div className="space-y-3">
        {/* Connection Status */}
        <div className="bg-blue-50 rounded-lg p-3 border border-blue-200">
          <div className="flex items-center justify-between">
            <span className="text-sm font-medium text-blue-800">Connection</span>
            <div className="flex items-center gap-2">
              <span className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`}></span>
              <span className={`text-sm font-semibold ${connected ? 'text-green-700' : 'text-red-700'}`}>
                {state}
              </span>
            </div>
          </div>
          {latency !== null && (
            <div className="text-xs text-blue-600 mt-1">
              Latency: {latency}ms
            </div>
          )}
        </div>

        {/* Battery */}
        <div className="bg-blue-50 rounded-lg p-3 border border-blue-200">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-blue-800">Battery</span>
            <span className="text-lg font-bold text-blue-900">
              {battery !== null ? `${battery}%` : '—'}
            </span>
          </div>
          {battery !== null && (
            <div className="w-full bg-blue-200 rounded-full h-2">
              <div
                className={`h-2 rounded-full ${
                  battery > 50 ? 'bg-green-500' : battery > 20 ? 'bg-yellow-500' : 'bg-red-500'
                }`}
                style={{ width: `${Math.min(100, Math.max(0, battery))}%` }}
              ></div>
            </div>
          )}
        </div>

        {/* Position */}
        <div className="bg-blue-50 rounded-lg p-3 border border-blue-200">
          <div className="text-sm font-medium text-blue-800 mb-2">Position</div>
          <div className="grid grid-cols-3 gap-2 text-xs">
            <div>
              <span className="text-blue-600">X:</span>
              <span className="ml-1 font-mono text-blue-900">{position.x.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-blue-600">Y:</span>
              <span className="ml-1 font-mono text-blue-900">{position.y.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-blue-600">Z:</span>
              <span className="ml-1 font-mono text-blue-900">{position.z.toFixed(2)}</span>
            </div>
          </div>
        </div>

        {/* Velocity */}
        <div className="bg-blue-50 rounded-lg p-3 border border-blue-200">
          <div className="text-sm font-medium text-blue-800 mb-2">Velocity</div>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div>
              <span className="text-blue-600">Linear:</span>
              <span className="ml-1 font-mono text-blue-900">{speed.toFixed(2)} m/s</span>
            </div>
            <div>
              <span className="text-blue-600">Angular:</span>
              <span className="ml-1 font-mono text-blue-900">{angularVel.z.toFixed(2)} rad/s</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}


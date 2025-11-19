import { useRosConnection, useBattery, useOdom, useIMURPY, useButton, useRobotState } from '../ros/hooks'
import { ROS_CONFIG } from '../ros/config'

export function TelemetryPanel() {
  const { connected, state, latency } = useRosConnection()
  const battery = useBattery()
  const odom = useOdom()
  const imuRpy = useIMURPY()
  const buttonPressed = useButton()
  const robotState = useRobotState()

  const voltageV = battery?.volts ?? null
  const percent = battery?.percent ?? null

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      <h2 className="text-lg font-semibold mb-3 text-slate-800">Status</h2>
      <div className="space-y-3">
        {/* Connection */}
        <div className="flex items-center justify-between">
          <span className="text-sm font-medium text-slate-700">Connection</span>
          <div className="flex items-center gap-2">
            <span className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'} ${connected ? 'animate-pulse' : ''}`}></span>
            <span className={`text-sm font-semibold ${connected ? 'text-green-700' : 'text-red-700'}`}>{state}</span>
          </div>
        </div>

        {/* Battery */}
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-sm font-medium text-slate-700">Battery</span>
            <span className="text-sm font-bold text-slate-900">{percent != null ? `${percent}%` : '—'}</span>
          </div>
          <div className="h-2 w-full bg-slate-200 rounded overflow-hidden">
            <div
              className="h-full bg-green-500 transition-all"
              style={{ width: `${Math.max(0, Math.min(100, percent ?? 0))}%` }}
            />
          </div>
          {voltageV != null && <div className="text-xs text-slate-500 mt-0.5">{voltageV.toFixed(2)} V</div>}
        </div>

        {/* Robot State */}
        {robotState && (
          <div className="flex items-center justify-between">
            <span className="text-sm font-medium text-slate-700">State</span>
            <span className="text-sm font-semibold text-slate-900">{robotState.replace(/_/g, ' ')}</span>
          </div>
        )}

        {/* IMU */}
        {imuRpy && (
          <div>
            <span className="text-sm font-medium text-slate-700 block mb-1">Orientation</span>
            <div className="grid grid-cols-3 gap-2 text-xs">
              <div className="text-slate-600">R: <span className="font-mono font-semibold">{(imuRpy.x * 180 / Math.PI).toFixed(0)}°</span></div>
              <div className="text-slate-600">P: <span className="font-mono font-semibold">{(imuRpy.y * 180 / Math.PI).toFixed(0)}°</span></div>
              <div className="text-slate-600">Y: <span className="font-mono font-semibold">{(imuRpy.z * 180 / Math.PI).toFixed(0)}°</span></div>
            </div>
          </div>
        )}

        {/* Button */}
        {buttonPressed !== null && (
          <div className="flex items-center justify-between">
            <span className="text-sm font-medium text-slate-700">Button</span>
            <span className={`text-sm font-semibold ${buttonPressed ? 'text-green-700' : 'text-slate-600'}`}>{buttonPressed ? 'Pressed' : 'Released'}</span>
          </div>
        )}
      </div>
    </section>
  )
}

export default TelemetryPanel

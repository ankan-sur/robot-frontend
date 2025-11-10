import { useRosConnection, useBattery, useOdom, useIMURPY, useButton, useRobotState } from '../ros/hooks'
import { ROS_CONFIG } from '../ros/config'

export function TelemetryPanel() {
  const { connected, state, latency } = useRosConnection()
  const battery = useBattery()
  const odom = useOdom()
  const imuRpy = useIMURPY()
  const buttonPressed = useButton()
  const robotState = useRobotState()
  const host = typeof window !== 'undefined' ? window.location.hostname : 'localhost'

  const velocity = (odom?.twist as any)?.twist?.linear || (odom?.twist as any)?.linear || { x: 0, y: 0, z: 0 }
  const angularVel = (odom?.twist as any)?.twist?.angular || (odom?.twist as any)?.angular || { x: 0, y: 0, z: 0 }
  const speed = Math.sqrt((velocity.x || 0) ** 2 + (velocity.y || 0) ** 2)

  // Battery values
  const voltageV = battery?.volts ?? null
  const percent = battery?.percent ?? null

  const getStateDisplay = () => {
    if (!robotState) {
      return { text: 'Unknown', color: 'bg-blue-400', textColor: 'text-blue-700', bg: 'bg-blue-50', border: 'border-blue-300' }
    }
    switch (robotState) {
      case 'idle':
        return { text: 'Idle', color: 'bg-blue-400', textColor: 'text-blue-700', bg: 'bg-blue-50', border: 'border-blue-300' }
      case 'responding_to_command':
        return { text: 'Responding to Command', color: 'bg-indigo-500', textColor: 'text-indigo-700', bg: 'bg-indigo-50', border: 'border-indigo-300' }
      case 'heading_to_charger':
        return { text: 'Heading to Charger', color: 'bg-sky-500', textColor: 'text-sky-700', bg: 'bg-sky-50', border: 'border-sky-300' }
      default:
        return { text: 'Unknown', color: 'bg-blue-400', textColor: 'text-blue-700', bg: 'bg-blue-50', border: 'border-blue-300' }
    }
  }

  const stateDisplay = getStateDisplay()

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Telemetry</h2>
      <div className="space-y-3">
        {/* Robot State */}
        <div className={`${stateDisplay.bg} rounded-lg p-3 border-2 ${stateDisplay.border}`}>
          <div className="flex items-center justify-between">
            <span className={`text-base font-medium ${stateDisplay.textColor}`}>Robot State</span>
            <div className="flex items-center gap-2">
              <span className={`w-3 h-3 rounded-full ${stateDisplay.color} ${robotState ? 'animate-pulse' : ''}`}></span>
              <span className={`text-base font-semibold ${stateDisplay.textColor}`}>
                {robotState ? stateDisplay.text : 'Signal unavailable'}
              </span>
            </div>
          </div>
          {!robotState && (
            <div className={`text-sm ${stateDisplay.textColor} mt-2 pt-2 border-t ${stateDisplay.border}`}>
              Waiting for /robot/state …
            </div>
          )}
        </div>

        {/* Connection */}
        <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-3 border-2 border-blue-300">
          <div className="flex items-center justify-between">
            <span className="text-base font-medium text-blue-800">Connection</span>
            <div className="flex items-center gap-2">
              <span className={`w-3 h-3 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'} ${connected ? 'animate-pulse' : ''}`}></span>
              <span className={`text-base font-semibold ${connected ? 'text-green-700' : 'text-red-700'}`}>{state}</span>
            </div>
          </div>
          {latency !== null && (
            <div className="text-sm text-blue-700 mt-1">Latency: <span className="font-mono">{latency}ms</span></div>
          )}
        </div>

        {/* Battery */}
        <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-3 border-2 border-blue-300">
          <div className="flex items-center justify-between mb-2">
            <span className="text-base font-medium text-blue-800">Battery</span>
            <div className="text-right">
              <div className="text-xl font-bold text-blue-900">{voltageV != null ? `${voltageV.toFixed(2)} V` : '—'}</div>
              <div className="text-xs text-blue-700">{percent != null ? `${percent}%` : 'percentage unavailable'}</div>
            </div>
          </div>
          <div className="h-3 w-full bg-blue-100 rounded overflow-hidden border border-blue-300">
            <div
              className="h-full bg-green-500 transition-all"
              style={{ width: `${Math.max(0, Math.min(100, percent ?? 0))}%` }}
            />
          </div>
        </div>

        {/* Network/Endpoints */}
        <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-3 border-2 border-blue-300">
          <div className="text-base font-medium text-blue-800 mb-2">Connection Info</div>
          <div className="text-sm space-y-1">
            <div><span className="text-blue-600">Host:</span> <span className="font-mono text-blue-900">{host}</span></div>
            <div><span className="text-blue-600">ROS Bridge:</span> <span className="font-mono text-blue-900">{ROS_CONFIG.rosbridgeUrl}</span></div>
            <div><span className="text-blue-600">Video Base:</span> <span className="font-mono text-blue-900">{ROS_CONFIG.videoBase}</span></div>
            <div><span className="text-blue-600">Camera Topic:</span> <span className="font-mono text-blue-900">{ROS_CONFIG.topics.camera}</span></div>
          </div>
        </div>

        {/* IMU Orientation */}
        {imuRpy && (
          <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-3 border-2 border-blue-300">
            <div className="text-base font-medium text-blue-800 mb-2">Orientation (IMU)</div>
            <div className="grid grid-cols-3 gap-2 text-sm">
              <div><span className="text-blue-600">Roll:</span><span className="ml-1 font-mono text-blue-900 font-semibold">{(imuRpy.x * 180 / Math.PI).toFixed(1)}°</span></div>
              <div><span className="text-blue-600">Pitch:</span><span className="ml-1 font-mono text-blue-900 font-semibold">{(imuRpy.y * 180 / Math.PI).toFixed(1)}°</span></div>
              <div><span className="text-blue-600">Yaw:</span><span className="ml-1 font-mono text-blue-900 font-semibold">{(imuRpy.z * 180 / Math.PI).toFixed(1)}°</span></div>
            </div>
          </div>
        )}

        {/* Button State */}
        {buttonPressed !== null && (
          <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-3 border-2 border-blue-300">
            <div className="flex items-center justify-between">
              <span className="text-base font-medium text-blue-800">Button</span>
              <div className="flex items-center gap-2">
                <span className={`w-3 h-3 rounded-full ${buttonPressed ? 'bg-green-500 animate-pulse' : 'bg-blue-400'}`}></span>
                <span className={`text-base font-semibold ${buttonPressed ? 'text-green-700' : 'text-blue-600'}`}>{buttonPressed ? 'Pressed' : 'Released'}</span>
              </div>
            </div>
          </div>
        )}
      </div>
    </section>
  )
}

export default TelemetryPanel

import { useRosConnection, useBattery, useOdom, useIMURPY, useButton, useRobotState, useCurrentCommand } from '../ros/hooks'

export function TelemetryPanel() {
  const { connected, state, latency } = useRosConnection()
  const battery = useBattery()
  const odom = useOdom()
  const imuRpy = useIMURPY()
  const buttonPressed = useButton()
  const robotState = useRobotState()
  const currentCommand = useCurrentCommand()

  const position = odom?.pose.pose.position || { x: 0, y: 0, z: 0 }
  const velocity = odom?.twist.linear || { x: 0, y: 0, z: 0 }
  const angularVel = odom?.twist.angular || { x: 0, y: 0, z: 0 }
  const speed = Math.sqrt(velocity.x ** 2 + velocity.y ** 2)

  // Get state display info
  const getStateDisplay = () => {
    if (!robotState) return null
    switch (robotState) {
      case 'idle':
        return { text: 'Idle', color: 'bg-slate-500', textColor: 'text-slate-700', bg: 'bg-slate-50', border: 'border-slate-300' }
      case 'responding_to_command':
        return { text: 'Responding to Command', color: 'bg-blue-500', textColor: 'text-blue-700', bg: 'bg-blue-50', border: 'border-blue-300' }
      case 'heading_to_charger':
        return { text: 'Heading to Charger', color: 'bg-amber-500', textColor: 'text-amber-700', bg: 'bg-amber-50', border: 'border-amber-300' }
      default:
        return { text: 'Unknown', color: 'bg-gray-500', textColor: 'text-gray-700', bg: 'bg-gray-50', border: 'border-gray-300' }
    }
  }
  
  const stateDisplay = getStateDisplay()

  return (
    <section className="rounded-lg border-2 border-cyan-400 bg-gradient-to-br from-white to-cyan-50 p-4 shadow-lg">
      <h2 className="text-lg font-semibold mb-3 bg-gradient-to-r from-cyan-600 to-blue-600 bg-clip-text text-transparent">Telemetry</h2>
      <div className="space-y-3">
        {/* Robot State */}
        {stateDisplay && (
          <div className={`${stateDisplay.bg} rounded-lg p-3 border-2 ${stateDisplay.border}`}>
            <div className="flex items-center justify-between">
              <span className={`text-sm font-medium ${stateDisplay.textColor}`}>Robot State</span>
              <div className="flex items-center gap-2">
                <span className={`w-3 h-3 rounded-full ${stateDisplay.color} animate-pulse`}></span>
                <span className={`text-sm font-semibold ${stateDisplay.textColor}`}>
                  {stateDisplay.text}
                </span>
              </div>
            </div>
            {currentCommand && (
              <div className={`text-xs ${stateDisplay.textColor} mt-2 pt-2 border-t ${stateDisplay.border}`}>
                <span className="font-medium">Current Command: </span>
                <span className="font-mono">{currentCommand}</span>
              </div>
            )}
          </div>
        )}

        {/* Connection Status */}
        <div className="bg-gradient-to-r from-emerald-50 to-teal-50 rounded-lg p-3 border-2 border-emerald-300">
          <div className="flex items-center justify-between">
            <span className="text-sm font-medium text-emerald-800">Connection</span>
            <div className="flex items-center gap-2">
              <span className={`w-3 h-3 rounded-full ${connected ? 'bg-emerald-500' : 'bg-red-500'} ${connected ? 'animate-pulse' : ''}`}></span>
              <span className={`text-sm font-semibold ${connected ? 'text-emerald-700' : 'text-red-700'}`}>
                {state}
              </span>
            </div>
          </div>
          {latency !== null && (
            <div className="text-xs text-emerald-700 mt-1">
              Latency: <span className="font-mono">{latency}ms</span>
            </div>
          )}
        </div>

        {/* Battery */}
        <div className="bg-gradient-to-r from-lime-50 to-green-50 rounded-lg p-3 border-2 border-lime-300">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-lime-800">Battery</span>
            <span className="text-lg font-bold text-lime-900">
              {battery !== null ? `${battery.toFixed(1)}%` : '—'}
            </span>
          </div>
          {battery !== null && (
            <div className="w-full bg-lime-200 rounded-full h-3 overflow-hidden">
              <div
                className={`h-3 rounded-full transition-all duration-300 ${
                  battery > 50 ? 'bg-gradient-to-r from-green-500 to-emerald-500' : 
                  battery > 20 ? 'bg-gradient-to-r from-yellow-400 to-amber-500' : 
                  'bg-gradient-to-r from-red-500 to-rose-600'
                }`}
                style={{ width: `${Math.min(100, Math.max(0, battery))}%` }}
              ></div>
            </div>
          )}
        </div>

        {/* Position */}
        <div className="bg-gradient-to-r from-indigo-50 to-purple-50 rounded-lg p-3 border-2 border-indigo-300">
          <div className="text-sm font-medium text-indigo-800 mb-2">Position</div>
          <div className="grid grid-cols-3 gap-2 text-xs">
            <div>
              <span className="text-indigo-600">X:</span>
              <span className="ml-1 font-mono text-indigo-900 font-semibold">{position.x.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-indigo-600">Y:</span>
              <span className="ml-1 font-mono text-indigo-900 font-semibold">{position.y.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-indigo-600">Z:</span>
              <span className="ml-1 font-mono text-indigo-900 font-semibold">{position.z.toFixed(2)}</span>
            </div>
          </div>
        </div>

        {/* Velocity */}
        <div className="bg-gradient-to-r from-violet-50 to-fuchsia-50 rounded-lg p-3 border-2 border-violet-300">
          <div className="text-sm font-medium text-violet-800 mb-2">Velocity</div>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div>
              <span className="text-violet-600">Linear:</span>
              <span className="ml-1 font-mono text-violet-900 font-semibold">{speed.toFixed(2)} m/s</span>
            </div>
            <div>
              <span className="text-violet-600">Angular:</span>
              <span className="ml-1 font-mono text-violet-900 font-semibold">{angularVel.z.toFixed(2)} rad/s</span>
            </div>
          </div>
        </div>

        {/* IMU Orientation */}
        {imuRpy && (
          <div className="bg-gradient-to-r from-rose-50 to-pink-50 rounded-lg p-3 border-2 border-rose-300">
            <div className="text-sm font-medium text-rose-800 mb-2">Orientation (IMU)</div>
            <div className="grid grid-cols-3 gap-2 text-xs">
              <div>
                <span className="text-rose-600">Roll:</span>
                <span className="ml-1 font-mono text-rose-900 font-semibold">{(imuRpy.x * 180 / Math.PI).toFixed(1)}°</span>
              </div>
              <div>
                <span className="text-rose-600">Pitch:</span>
                <span className="ml-1 font-mono text-rose-900 font-semibold">{(imuRpy.y * 180 / Math.PI).toFixed(1)}°</span>
              </div>
              <div>
                <span className="text-rose-600">Yaw:</span>
                <span className="ml-1 font-mono text-rose-900 font-semibold">{(imuRpy.z * 180 / Math.PI).toFixed(1)}°</span>
              </div>
            </div>
          </div>
        )}

        {/* Button State */}
        {buttonPressed !== null && (
          <div className="bg-gradient-to-r from-sky-50 to-cyan-50 rounded-lg p-3 border-2 border-sky-300">
            <div className="flex items-center justify-between">
              <span className="text-sm font-medium text-sky-800">Button</span>
              <div className="flex items-center gap-2">
                <span className={`w-3 h-3 rounded-full ${buttonPressed ? 'bg-emerald-500 animate-pulse' : 'bg-slate-400'}`}></span>
                <span className={`text-sm font-semibold ${buttonPressed ? 'text-emerald-700' : 'text-slate-600'}`}>
                  {buttonPressed ? 'Pressed' : 'Released'}
                </span>
              </div>
            </div>
          </div>
        )}
      </div>
    </section>
  )
}


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
    if (!robotState) {
      // Default placeholder when no state
      return { text: 'Unknown', color: 'bg-purple-400', textColor: 'text-purple-700', bg: 'bg-purple-50', border: 'border-purple-300' }
    }
    switch (robotState) {
      case 'idle':
        return { text: 'Idle', color: 'bg-purple-400', textColor: 'text-purple-700', bg: 'bg-purple-50', border: 'border-purple-300' }
      case 'responding_to_command':
        return { text: 'Responding to Command', color: 'bg-pink-500', textColor: 'text-pink-700', bg: 'bg-pink-50', border: 'border-pink-300' }
      case 'heading_to_charger':
        return { text: 'Heading to Charger', color: 'bg-rose-500', textColor: 'text-rose-700', bg: 'bg-rose-50', border: 'border-rose-300' }
      default:
        return { text: 'Unknown', color: 'bg-purple-400', textColor: 'text-purple-700', bg: 'bg-purple-50', border: 'border-purple-300' }
    }
  }
  
  const stateDisplay = getStateDisplay()

  return (
    <section className="rounded-lg border-2 border-purple-400 bg-gradient-to-br from-white to-purple-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent">Telemetry</h2>
      <div className="space-y-3">
        {/* Robot State - Always show with placeholder */}
        <div className={`${stateDisplay.bg} rounded-lg p-3 border-2 ${stateDisplay.border}`}>
          <div className="flex items-center justify-between">
            <span className={`text-base font-medium ${stateDisplay.textColor}`}>Robot State</span>
            <div className="flex items-center gap-2">
              <span className={`w-3 h-3 rounded-full ${stateDisplay.color} ${robotState ? 'animate-pulse' : ''}`}></span>
              <span className={`text-base font-semibold ${stateDisplay.textColor}`}>
                {robotState ? stateDisplay.text : '—'}
              </span>
            </div>
          </div>
          <div className={`text-sm ${stateDisplay.textColor} mt-2 pt-2 border-t ${stateDisplay.border}`}>
            <span className="font-medium">Current Command: </span>
            <span className="font-mono">{currentCommand || '—'}</span>
          </div>
        </div>

        {/* Connection Status */}
        <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
          <div className="flex items-center justify-between">
            <span className="text-base font-medium text-purple-800">Connection</span>
            <div className="flex items-center gap-2">
              <span className={`w-3 h-3 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'} ${connected ? 'animate-pulse' : ''}`}></span>
              <span className={`text-base font-semibold ${connected ? 'text-green-700' : 'text-red-700'}`}>
                {state}
              </span>
            </div>
          </div>
          {latency !== null && (
            <div className="text-sm text-purple-700 mt-1">
              Latency: <span className="font-mono">{latency}ms</span>
            </div>
          )}
        </div>

        {/* Battery */}
        <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
          <div className="flex items-center justify-between mb-2">
            <span className="text-base font-medium text-purple-800">Battery</span>
            <span className="text-xl font-bold text-purple-900">
              {battery !== null ? `${battery.toFixed(1)}%` : '—'}
            </span>
          </div>
          {battery !== null && (
            <div className="w-full bg-purple-200 rounded-full h-3 overflow-hidden">
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
        <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
          <div className="text-base font-medium text-purple-800 mb-2">Position</div>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div>
              <span className="text-purple-600">X:</span>
              <span className="ml-1 font-mono text-purple-900 font-semibold">{position.x.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-purple-600">Y:</span>
              <span className="ml-1 font-mono text-purple-900 font-semibold">{position.y.toFixed(2)}</span>
            </div>
            <div>
              <span className="text-purple-600">Z:</span>
              <span className="ml-1 font-mono text-purple-900 font-semibold">{position.z.toFixed(2)}</span>
            </div>
          </div>
        </div>

        {/* Velocity */}
        <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
          <div className="text-base font-medium text-purple-800 mb-2">Velocity</div>
          <div className="grid grid-cols-2 gap-2 text-sm">
            <div>
              <span className="text-purple-600">Linear:</span>
              <span className="ml-1 font-mono text-purple-900 font-semibold">{speed.toFixed(2)} m/s</span>
            </div>
            <div>
              <span className="text-purple-600">Angular:</span>
              <span className="ml-1 font-mono text-purple-900 font-semibold">{angularVel.z.toFixed(2)} rad/s</span>
            </div>
          </div>
        </div>

        {/* IMU Orientation */}
        {imuRpy && (
          <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
            <div className="text-base font-medium text-purple-800 mb-2">Orientation (IMU)</div>
            <div className="grid grid-cols-3 gap-2 text-sm">
              <div>
                <span className="text-purple-600">Roll:</span>
                <span className="ml-1 font-mono text-purple-900 font-semibold">{(imuRpy.x * 180 / Math.PI).toFixed(1)}°</span>
              </div>
              <div>
                <span className="text-purple-600">Pitch:</span>
                <span className="ml-1 font-mono text-purple-900 font-semibold">{(imuRpy.y * 180 / Math.PI).toFixed(1)}°</span>
              </div>
              <div>
                <span className="text-purple-600">Yaw:</span>
                <span className="ml-1 font-mono text-purple-900 font-semibold">{(imuRpy.z * 180 / Math.PI).toFixed(1)}°</span>
              </div>
            </div>
          </div>
        )}

        {/* Button State */}
        {buttonPressed !== null && (
          <div className="bg-gradient-to-r from-purple-50 to-pink-50 rounded-lg p-3 border-2 border-purple-300">
            <div className="flex items-center justify-between">
              <span className="text-base font-medium text-purple-800">Button</span>
              <div className="flex items-center gap-2">
                <span className={`w-3 h-3 rounded-full ${buttonPressed ? 'bg-green-500 animate-pulse' : 'bg-purple-400'}`}></span>
                <span className={`text-base font-semibold ${buttonPressed ? 'text-green-700' : 'text-purple-600'}`}>
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


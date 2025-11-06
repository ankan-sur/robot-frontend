import { useRef, useState } from 'react'
import { changeMap, useAvailableMaps } from '../ros/hooks'
import { topics } from '../ros/ros'

type Props = {
  onMapChange?: (mapName: string) => void
}

export function DebugPanel({ onMapChange }: Props) {
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const availableMaps = useAvailableMaps()

  const handleChangeMap = async () => {
    if (!selectedMap) return

    setLoading(true)
    setError(null)

    try {
      await changeMap(selectedMap)
      onMapChange?.(selectedMap)
      alert(`Map changed to: ${selectedMap}`)
    } catch (err: any) {
      setError(err.message || 'Failed to change map')
      console.error('Map change error:', err)
    } finally {
      setLoading(false)
    }
  }

  const hasMaps = availableMaps.length > 0

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Debug Panel</h2>
      <div className="space-y-4">
        {/* Teleop (moved here for larger layout) */}
        <div className="rounded-lg border-2 border-blue-300 bg-white p-4">
          <div className="text-base font-medium text-blue-800 mb-3">Teleop</div>
          <TeleopBlock />
        </div>
        <div>
          <label className="block text-base font-medium text-blue-700 mb-2" htmlFor="map-select">
            Select Map
          </label>
          <select
            id="map-select"
            className={`w-full rounded-lg border-2 px-3 py-2 text-base focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all ${
              hasMaps 
                ? 'border-blue-300 bg-white text-blue-900 hover:border-blue-400' 
                : 'border-gray-300 bg-gray-100 text-gray-400'
            }`}
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
            disabled={loading || !hasMaps}
          >
            <option value="">
              {hasMaps ? 'Choose a map...' : 'NA - No maps available'}
            </option>
            {availableMaps.map((map) => (
              <option key={map} value={map}>
                {map}
              </option>
            ))}
          </select>
          {!hasMaps && (
            <div className="text-xs text-gray-500 mt-1">
              Waiting for maps from ROS...
            </div>
          )}
        </div>
        <button
          onClick={handleChangeMap}
          disabled={!selectedMap || loading || !hasMaps}
          className="w-full px-4 py-2 rounded-lg bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-700 hover:to-indigo-700 text-white font-medium text-base disabled:opacity-50 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg"
        >
          {loading ? 'Changing...' : 'Change Map'}
        </button>
        {error && (
          <div className="text-base text-red-700 bg-red-50 p-2 rounded-lg border-2 border-red-300 font-medium">
            {error}
          </div>
        )}
        <div className="text-sm text-blue-700 mt-2">
          Maps from: <code className="bg-blue-100 px-2 py-0.5 rounded font-mono">ros2_ws/src/slam/maps</code>
        </div>
      </div>
    </section>
  )
}

function TeleopBlock() {
  const [lin, setLin] = useState(0.3)
  const [ang, setAng] = useState(1.0)
  const held = useRef({ vx: 0, wz: 0 })
  const intervalRef = useRef<number | null>(null)

  const publishTwist = () => topics.cmdVel.publish({
    linear: { x: held.current.vx, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: held.current.wz },
  } as any)

  const startLoop = () => {
    if (intervalRef.current != null) return
    // Publish at ~20 Hz while a button is held
    intervalRef.current = window.setInterval(() => {
      publishTwist()
    }, 50)
  }

  const stopLoopIfIdle = () => {
    if (held.current.vx === 0 && held.current.wz === 0 && intervalRef.current != null) {
      window.clearInterval(intervalRef.current)
      intervalRef.current = null
    }
  }

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 items-center">
        <div className="flex items-center gap-3">
          <label className="text-sm text-slate-600 w-20">Linear</label>
          <input
            className="flex-1"
            type="range"
            min="0"
            max="1.5"
            step="0.05"
            value={lin}
            onChange={e => {
              const v = +e.target.value
              setLin(v)
              // If currently driving forward/back, update speed live
              if (held.current.vx !== 0) {
                held.current.vx = Math.sign(held.current.vx) * v
              }
            }}
          />
          <div className="w-14 text-right font-mono text-sm text-slate-700">{lin.toFixed(2)}</div>
        </div>
        <div className="flex items-center gap-3">
          <label className="text-sm text-slate-600 w-20">Angular</label>
          <input
            className="flex-1"
            type="range"
            min="0"
            max="2.5"
            step="0.05"
            value={ang}
            onChange={e => {
              const v = +e.target.value
              setAng(v)
              // If currently turning, update speed live
              if (held.current.wz !== 0) {
                held.current.wz = Math.sign(held.current.wz) * v
              }
            }}
          />
          <div className="w-14 text-right font-mono text-sm text-slate-700">{ang.toFixed(2)}</div>
        </div>
      </div>
      <div className="flex flex-wrap gap-2">
        <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
          onMouseDown={() => { held.current.vx = lin; startLoop() }}
          onMouseUp={() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() }}
          onTouchStart={(e) => { e.preventDefault(); held.current.vx = lin; startLoop() }}
          onTouchEnd={(e) => { e.preventDefault(); held.current.vx = 0; publishTwist(); stopLoopIfIdle() }}
        >Fwd</button>
        <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
          onMouseDown={() => { held.current.vx = -lin; startLoop() }}
          onMouseUp={() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() }}
          onTouchStart={(e) => { e.preventDefault(); held.current.vx = -lin; startLoop() }}
          onTouchEnd={(e) => { e.preventDefault(); held.current.vx = 0; publishTwist(); stopLoopIfIdle() }}
        >Rev</button>
        <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
          onMouseDown={() => { held.current.wz = ang; startLoop() }}
          onMouseUp={() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() }}
          onTouchStart={(e) => { e.preventDefault(); held.current.wz = ang; startLoop() }}
          onTouchEnd={(e) => { e.preventDefault(); held.current.wz = 0; publishTwist(); stopLoopIfIdle() }}
        >Left</button>
        <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
          onMouseDown={() => { held.current.wz = -ang; startLoop() }}
          onMouseUp={() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() }}
          onTouchStart={(e) => { e.preventDefault(); held.current.wz = -ang; startLoop() }}
          onTouchEnd={(e) => { e.preventDefault(); held.current.wz = 0; publishTwist(); stopLoopIfIdle() }}
        >Right</button>
        <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
          onClick={() => { held.current = { vx: 0, wz: 0 }; publishTwist(); stopLoopIfIdle() }}
        >Stop</button>
      </div>
    </div>
  )
}


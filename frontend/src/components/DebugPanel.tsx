import { useState } from 'react'
import { changeMap, useAvailableMaps } from '../ros/hooks'
import { RwtPanel } from './RwtPanel'
import { ROS_CONFIG } from '../ros/config'

type Props = {
  onMapChange?: (mapName: string) => void
}

export function DebugPanel({ onMapChange }: Props) {
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [loading, setLoading] = useState(false)
  const [status, setStatus] = useState<{ kind: 'success' | 'error'; text: string } | null>(null)
  const availableMaps = useAvailableMaps()

  const handleChangeMap = async () => {
    if (!selectedMap) return

    setLoading(true)
    setStatus(null)

    try {
      await changeMap(selectedMap)
      onMapChange?.(selectedMap)
      setStatus({ kind: 'success', text: `Map changed to ${selectedMap}` })
    } catch (err: any) {
      setStatus({ kind: 'error', text: err?.message || 'Failed to change map' })
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
        <RwtPanel
          title="Teleop"
          src={ROS_CONFIG.rwt.teleop}
          description="Expose visualization_rwt's teleop widget and set VITE_RWT_TELEOP_URL so the dashboard can embed it."
          heightClass="h-80"
        />
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
        {status && (
          <div className={`text-base p-2 rounded-lg border-2 font-medium ${status.kind === 'success' ? 'text-green-700 bg-green-50 border-green-200' : 'text-red-700 bg-red-50 border-red-200'}`}>
            {status.text}
          </div>
        )}
        <div className="text-sm text-blue-700 mt-2 space-y-2">
          <div>
            Maps from: <code className="bg-blue-100 px-2 py-0.5 rounded font-mono">ros2_ws/src/slam/maps</code>
          </div>
          <div className="flex flex-wrap gap-2">
            {ROS_CONFIG.rwt.rosboard && (
              <a
                href={ROS_CONFIG.rwt.rosboard}
                target="_blank"
                rel="noreferrer"
                className="px-3 py-1.5 text-sm rounded bg-slate-900 text-white hover:bg-slate-800"
              >
                Open Rosboard
              </a>
            )}
            {ROS_CONFIG.rwt.rosTool && (
              <a
                href={ROS_CONFIG.rwt.rosTool}
                target="_blank"
                rel="noreferrer"
                className="px-3 py-1.5 text-sm rounded bg-indigo-600 text-white hover:bg-indigo-500"
              >
                Open ROS Tool
              </a>
            )}
          </div>
        </div>
      </div>
    </section>
  )
}


import { useState } from 'react'
import { changeMap } from '../ros/hooks'

// Common map names - adjust based on your actual maps in ros2_ws/src/slam/maps
const AVAILABLE_MAPS = [
  'map.yaml',
  'warehouse.yaml',
  'lab1.yaml',
  'lab2.yaml',
  'hospital.yaml',
]

type Props = {
  onMapChange?: (mapName: string) => void
}

export function DebugPanel({ onMapChange }: Props) {
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

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

  return (
    <section className="rounded-lg border-2 border-blue-300 bg-white p-4 shadow-md">
      <h2 className="text-lg font-semibold mb-3 text-blue-900">Debug Panel</h2>
      <div className="space-y-3">
        <div>
          <label className="block text-sm font-medium text-blue-800 mb-1" htmlFor="map-select">
            Select Map
          </label>
          <select
            id="map-select"
            className="w-full rounded border-2 border-blue-200 bg-white px-3 py-2 text-sm text-blue-900 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
            disabled={loading}
          >
            <option value="">Choose a map...</option>
            {AVAILABLE_MAPS.map((map) => (
              <option key={map} value={map}>
                {map}
              </option>
            ))}
          </select>
        </div>
        <button
          onClick={handleChangeMap}
          disabled={!selectedMap || loading}
          className="w-full px-4 py-2 rounded bg-blue-600 hover:bg-blue-700 text-white font-medium disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        >
          {loading ? 'Changing...' : 'Change Map'}
        </button>
        {error && (
          <div className="text-sm text-red-600 bg-red-50 p-2 rounded border border-red-200">
            {error}
          </div>
        )}
        <div className="text-xs text-blue-600 mt-2">
          Maps from: <code className="bg-blue-100 px-1 rounded">ros2_ws/src/slam/maps</code>
        </div>
      </div>
    </section>
  )
}


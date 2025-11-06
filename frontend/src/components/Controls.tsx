import { useState } from 'react'
import { usePointsOfInterest, PointOfInterest } from '../ros/hooks'

type Props = {
  goToLab: (poi: PointOfInterest) => void
  onStop: () => void
  disabledMove?: boolean
  disabledStop?: boolean
  controlAllowed?: boolean
}

export function Controls({ goToLab, onStop, disabledMove, disabledStop, controlAllowed }: Props) {
  const pois = usePointsOfInterest()
  const [selectedPoi, setSelectedPoi] = useState<string>('')
  
  const selectedPoiData = pois.find(p => `${p.x},${p.y}` === selectedPoi)

  return (
    <section className="rounded-lg border-2 border-blue-300 bg-white p-4 shadow-md">
      <h2 className="text-lg font-semibold mb-3 text-blue-900">Controls</h2>
      <div className="space-y-4">
        <div>
          <label className="block text-sm font-medium text-blue-800 mb-2" htmlFor="poi-select">
            Destination
          </label>
          <div className="flex items-center gap-2">
            <select
              id="poi-select"
              className="flex-1 rounded border-2 border-blue-200 bg-white px-3 py-2 text-sm text-blue-900 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
              value={selectedPoi}
              onChange={(e) => setSelectedPoi(e.target.value)}
            >
              <option value="">Select destination…</option>
              {pois.map((poi, idx) => (
                <option key={idx} value={`${poi.x},${poi.y}`}>
                  {poi.name} ({poi.x.toFixed(1)}, {poi.y.toFixed(1)})
                </option>
              ))}
            </select>
            <button
              onClick={() => {
                if (selectedPoiData) goToLab(selectedPoiData)
              }}
              disabled={disabledMove || !selectedPoiData}
              title={disabledMove ? 'Controls not available' : !selectedPoiData ? 'Select a destination' : 'Navigate to destination'}
              className="px-4 py-2 rounded bg-blue-600 hover:bg-blue-700 text-white font-medium disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
            >
              Go
            </button>
          </div>
        </div>
        <div className={`text-xs p-2 rounded ${controlAllowed ? 'bg-green-50 text-green-700 border border-green-200' : 'bg-yellow-50 text-yellow-700 border border-yellow-200'}`}>
          {controlAllowed ? '✓ You have control' : '⚠ Controls disabled until robot grants control'}
        </div>
        <button
          onClick={onStop}
          disabled={disabledStop}
          title={disabledStop ? 'Stop disabled' : 'Immediately stop the robot'}
          className="w-full px-4 py-2 rounded bg-red-600 hover:bg-red-700 text-white font-semibold disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        >
          Emergency Stop
        </button>
      </div>
    </section>
  )
}

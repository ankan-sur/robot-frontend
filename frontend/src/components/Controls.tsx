import { useState } from 'react'
import { usePointsOfInterest, PointOfInterest, useRobotState } from '../ros/hooks'

type Props = {
  goToLab: (poi: PointOfInterest) => void
  onStop: () => void
  disabledMove?: boolean
  controlAllowed?: boolean
}

export function Controls({ goToLab, onStop, disabledMove, controlAllowed }: Props) {
  const pois = usePointsOfInterest()
  const robotState = useRobotState()
  const [selectedPoi, setSelectedPoi] = useState<string>('')
  
  const selectedPoiData = pois.find(p => `${p.x},${p.y}` === selectedPoi)
  const hasPois = pois.length > 0
  // Only enable stop if robot is responding to a command
  const isRespondingToCommand = robotState === 'responding_to_command'

  return (
    <section className="rounded-lg border-2 border-purple-400 bg-gradient-to-br from-white to-purple-50 p-4 shadow-lg">
      <h2 className="text-lg font-semibold mb-3 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent">Controls</h2>
      <div className="space-y-4">
        <div>
          <label className="block text-sm font-medium text-purple-700 mb-2" htmlFor="poi-select">
            Destination
          </label>
          <div className="flex items-center gap-2">
            <select
              id="poi-select"
              className={`flex-1 rounded-lg border-2 px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-purple-500 transition-all ${
                hasPois
                  ? 'border-purple-300 bg-white text-purple-900 hover:border-purple-400'
                  : 'border-gray-300 bg-gray-100 text-gray-400'
              }`}
              value={selectedPoi}
              onChange={(e) => setSelectedPoi(e.target.value)}
              disabled={!hasPois}
            >
              <option value="">
                {hasPois ? 'Select destination…' : 'NA - No destinations available'}
              </option>
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
              disabled={disabledMove || !selectedPoiData || !hasPois}
              title={disabledMove ? 'Controls not available' : !hasPois ? 'No destinations available' : !selectedPoiData ? 'Select a destination' : 'Navigate to destination'}
              className="px-4 py-2 rounded-lg bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-700 hover:to-pink-700 text-white font-medium disabled:opacity-50 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg"
            >
              Go
            </button>
          </div>
          {!hasPois && (
            <div className="text-xs text-gray-500 mt-1">
              Waiting for POIs from ROS...
            </div>
          )}
        </div>
        <div className={`text-xs p-2 rounded-lg border-2 ${controlAllowed ? 'bg-emerald-50 text-emerald-800 border-emerald-300' : 'bg-amber-50 text-amber-800 border-amber-300'}`}>
          {controlAllowed ? '✓ You have control' : '⚠ Controls disabled until robot grants control'}
        </div>
        <button
          onClick={onStop}
          disabled={!isRespondingToCommand}
          title={!isRespondingToCommand ? 'Stop only available when robot is responding to a command' : 'Stop the current command'}
          className="w-full px-4 py-2 rounded-lg bg-gradient-to-r from-orange-500 to-red-600 hover:from-orange-600 hover:to-red-700 text-white font-semibold disabled:opacity-40 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg disabled:hover:shadow-md"
        >
          Stop
        </button>
      </div>
    </section>
  )
}

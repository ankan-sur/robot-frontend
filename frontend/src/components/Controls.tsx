import { useState, useEffect } from 'react'
import { useRobotState, usePointsOfInterest, PointOfInterest } from '../ros/hooks'
import { getMode } from '../ros/services'

type Props = {
  goToLab: (poi: PointOfInterest) => void
  onStop: () => void
  disabledMove?: boolean
}

export function Controls({ goToLab, onStop, disabledMove }: Props) {
  const robotState = useRobotState()
  const [selectedPoi, setSelectedPoi] = useState<string>('')
  const [currentMode, setCurrentMode] = useState<string | null>(null)
  const pois = usePointsOfInterest()
  const selectedPoiData = pois.find(p => `${p.x},${p.y}` === selectedPoi)
  const hasPois = pois.length > 0
  
  // Fetch current mode periodically
  useEffect(() => {
    const fetchMode = async () => {
      try {
        const res = await getMode()
        setCurrentMode(res?.mode || null)
      } catch {
        setCurrentMode(null)
      }
    }
    fetchMode()
    const interval = setInterval(fetchMode, 5000) // refresh every 5s
    return () => clearInterval(interval)
  }, [])
  
  const inSlamMode = currentMode === 'slam'
  const canIssueCommands = !(disabledMove) && !inSlamMode // disable navigation in SLAM mode
  const stopEnabled = canIssueCommands && (robotState ? robotState !== 'idle' : true)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Controls</h2>
        {currentMode && (
          <span className={`text-xs px-2 py-1 rounded-full font-semibold ${
            currentMode === 'slam' ? 'bg-yellow-200 text-yellow-800' :
            currentMode === 'localization' ? 'bg-blue-200 text-blue-800' :
            'bg-gray-200 text-gray-700'
          }`}>
            {currentMode.toUpperCase()}
          </span>
        )}
      </div>
      {inSlamMode && (
        <div className="mb-3 bg-yellow-100 border border-yellow-400 rounded-lg px-3 py-2 text-sm text-yellow-900">
          ⚠️ Navigation disabled while mapping
        </div>
      )}
      <div className="space-y-4">
        <div>
          <label className="block text-base font-medium text-blue-700 mb-2" htmlFor="poi-select">
            Destination
          </label>
          <div className="flex items-center gap-2">
            <select
              id="poi-select"
              className={`flex-1 rounded-lg border-2 px-3 py-2 text-base focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all ${
                hasPois
                  ? 'border-blue-300 bg-white text-blue-900 hover:border-blue-400'
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
              disabled={!canIssueCommands || !selectedPoiData || !hasPois}
              title={
                !canIssueCommands
                  ? 'Controls not available'
                  : !hasPois
                    ? 'No destinations available'
                    : !selectedPoiData
                      ? 'Select a destination'
                      : 'Navigate to destination'
              }
              className="px-4 py-2 rounded-lg bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-700 hover:to-indigo-700 text-white font-medium text-base disabled:opacity-50 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg"
            >
              Go
            </button>
          </div>
          {!hasPois && (
            <div className="text-sm text-gray-500 mt-1">
              Waiting for POIs from ROS...
            </div>
          )}
        </div>
        {/* Removed "You have control" banner per request */}
        <button
          onClick={onStop}
          disabled={!stopEnabled}
          title={stopEnabled ? 'Stop the current command' : 'Stop is only available when connected'}
          className="w-full px-4 py-2 rounded-lg bg-gradient-to-r from-orange-500 to-red-600 hover:from-orange-600 hover:to-red-700 text-white font-semibold text-base disabled:opacity-40 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg disabled:hover:shadow-md"
        >
          Stop
        </button>

      </div>
    </section>
  )
}

import { useRef, useState } from 'react'
import { usePointsOfInterest, PointOfInterest, useRobotState } from '../ros/hooks'
import { topics } from '../ros/ros'

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
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Controls</h2>
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
              disabled={disabledMove || !selectedPoiData || !hasPois}
              title={disabledMove ? 'Controls not available' : !hasPois ? 'No destinations available' : !selectedPoiData ? 'Select a destination' : 'Navigate to destination'}
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
        <div className={`text-sm p-2 rounded-lg border-2 ${controlAllowed ? 'bg-green-50 text-green-800 border-green-300' : 'bg-amber-50 text-amber-800 border-amber-300'}`}>
          {controlAllowed ? '✓ You have control' : '⚠ Controls disabled until robot grants control'}
        </div>
        <button
          onClick={onStop}
          disabled={!isRespondingToCommand}
          title={!isRespondingToCommand ? 'Stop only available when robot is responding to a command' : 'Stop the current command'}
          className="w-full px-4 py-2 rounded-lg bg-gradient-to-r from-orange-500 to-red-600 hover:from-orange-600 hover:to-red-700 text-white font-semibold text-base disabled:opacity-40 disabled:cursor-not-allowed transition-all shadow-md hover:shadow-lg disabled:hover:shadow-md"
        >
          Stop
        </button>

        {/* Teleop (Twist) */}
        <div className="rounded-lg border-2 border-blue-300 bg-white p-3">
          <div className="text-base font-medium text-blue-800 mb-2">Teleop</div>
          <TeleopBlock />
        </div>

        {/* Command Buttons via /ui/cmd */}
        <div className="rounded-lg border-2 border-blue-300 bg-white p-3">
          <div className="text-base font-medium text-blue-800 mb-2">Quick Commands</div>
          <div className="flex flex-wrap gap-2">
            <button className="px-3 py-1.5 border rounded bg-blue-600 hover:bg-blue-700 text-white" onClick={() => topics.uiCmd.publish({ data: JSON.stringify({ type: 'goto', target: 'lab1' }) } as any)}>Go Lab 1</button>
            <button className="px-3 py-1.5 border rounded bg-blue-600 hover:bg-blue-700 text-white" onClick={() => topics.uiCmd.publish({ data: JSON.stringify({ type: 'goto', target: 'lab2' }) } as any)}>Go Lab 2</button>
            <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onClick={() => topics.uiCancel.publish({ data: JSON.stringify({ id: 'current' }) } as any)}>Cancel</button>
          </div>
        </div>
      </div>
    </section>
  )
}

function TeleopBlock() {
  const [lin, setLin] = useState(0.2)
  const [ang, setAng] = useState(0.8)
  const held = useRef({ vx: 0, wz: 0 })

  const publishTwist = () => topics.cmdVel.publish({
    linear: { x: held.current.vx, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: held.current.wz },
  } as any)

  return (
    <div className="space-y-3">
      <div className="flex items-center gap-3">
        <label className="text-sm text-slate-600">Linear</label>
        <input type="range" min="0.05" max="0.6" step="0.05" value={lin} onChange={e => setLin(+e.target.value)} />
        <label className="text-sm text-slate-600">Angular</label>
        <input type="range" min="0.2" max="1.2" step="0.05" value={ang} onChange={e => setAng(+e.target.value)} />
      </div>
      <div className="flex flex-wrap gap-2">
        <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onMouseDown={() => { held.current.vx = lin; publishTwist() }} onMouseUp={() => { held.current.vx = 0; publishTwist() }}>Fwd</button>
        <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onMouseDown={() => { held.current.vx = -lin; publishTwist() }} onMouseUp={() => { held.current.vx = 0; publishTwist() }}>Rev</button>
        <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onMouseDown={() => { held.current.wz = ang; publishTwist() }} onMouseUp={() => { held.current.wz = 0; publishTwist() }}>Left</button>
        <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onMouseDown={() => { held.current.wz = -ang; publishTwist() }} onMouseUp={() => { held.current.wz = 0; publishTwist() }}>Right</button>
        <button className="px-3 py-1.5 border rounded bg-slate-100 hover:bg-slate-200" onClick={() => { held.current = { vx: 0, wz: 0 }; publishTwist() }}>Stop</button>
      </div>
    </div>
  )
}

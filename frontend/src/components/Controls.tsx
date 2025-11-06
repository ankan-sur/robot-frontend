import { useState } from 'react'

type Props = {
  goToLab: (n: number) => void
  onStop: () => void
  disabledMove?: boolean
  disabledStop?: boolean
  controlAllowed?: boolean
}

export function Controls({ goToLab, onStop, disabledMove, disabledStop, controlAllowed }: Props) {
  const [selectedLab, setSelectedLab] = useState<number | ''>('')
  
  return (
    <section className="rounded-lg border border-slate-200 bg-white p-4">
      <h2 className="text-lg font-medium mb-3 text-slate-800">Controls</h2>
      <div className="space-y-4">
        <div className="flex items-center gap-3 mb-2">
          <label className="text-sm text-slate-600" htmlFor="lab-select">Destination</label>
          <select
            id="lab-select"
            className="flex-1 rounded border border-slate-300 bg-white px-3 py-2 text-sm text-slate-800 focus:outline-none focus:ring-2 focus:ring-sky-300"
            value={selectedLab}
            onChange={(e) => {
              const v = e.target.value
              setSelectedLab(v === '' ? '' : Number(v))
            }}
          >
            <option value="">Select destination…</option>
            <option value={1}>Lab 1</option>
            <option value={2}>Lab 2</option>
          </select>
          <button
            onClick={() => { if (selectedLab !== '') goToLab(selectedLab as number) }}
            disabled={disabledMove || selectedLab === ''}
            title={disabledMove ? 'Controls not available' : selectedLab === '' ? 'Select a destination' : 'Send robot to destination'}
            className="px-4 py-2 rounded bg-sky-700 hover:bg-sky-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
          >
            Go
          </button>
        </div>
        <div className="text-xs text-slate-500 mb-2">
          {controlAllowed ? 'You have control.' : 'Controls are currently disabled until the robot grants control.'}
        </div>
        <div className="flex gap-2">
          <button
            onClick={onStop}
            disabled={disabledStop}
            title={disabledStop ? 'Stop disabled' : 'Immediately stop the robot'}
            className="px-4 py-2 rounded bg-rose-600 hover:bg-rose-500 text-white disabled:opacity-50 disabled:cursor-not-allowed w-full"
          >
            Stop
          </button>
        </div>
      </div>
    </section>
  )
}


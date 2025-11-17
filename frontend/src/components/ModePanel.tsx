import { useState } from 'react'
import { setMode, cancelNavigation } from '../ros/services'
import { useRobotState, useCmdVel, useNavStatus } from '../ros/hooks'

export default function ModePanel() {
  const robotState = useRobotState()
  const { stop: emergencyStop } = useCmdVel()
  const nav = useNavStatus()

  const [busy, setBusy] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)

  const changeMode = async (mode: string) => {
    setBusy(`Setting mode: ${mode}`)
    setError(null)
    try {
      await setMode(mode)
    } catch (e: any) {
      setError(e?.message || `Failed to set mode to ${mode}`)
    } finally {
      setBusy(null)
    }
  }

  const handleEmergency = async () => {
    setBusy('Emergency stop')
    setError(null)
    try {
      // Stop robot motion immediately
      emergencyStop()
      // Try to set robot into safe idle mode
      try { await setMode('idle') } catch {}
    } catch (e: any) {
      setError(e?.message || 'Emergency stop failed')
    } finally {
      setBusy(null)
    }
  }

  const handleCancelNav = async () => {
    setBusy('Canceling navigation…')
    setError(null)
    try {
      await cancelNavigation(nav?.id as any)
    } catch (e: any) {
      setError(e?.message || 'Cancel failed')
    } finally {
      setBusy(null)
    }
  }

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Modes</h2>

      {error && (<div className="mb-2 p-2 text-sm text-red-700 bg-red-50 rounded border border-red-200">{error}</div>)}
      {busy && (<div className="mb-2 p-2 text-sm text-blue-700 bg-blue-50 rounded border border-blue-200">{busy}</div>)}

      <div className="grid grid-cols-2 gap-2">
        <button onClick={() => changeMode('manual')} className="px-3 py-2 rounded bg-indigo-600 text-white">Manual (Teleop)</button>
        <button onClick={() => changeMode('autonomous')} className="px-3 py-2 rounded bg-emerald-600 text-white">Autonomous</button>
        <button onClick={() => changeMode('charging')} className="px-3 py-2 rounded bg-yellow-500 text-white">Go to Charger</button>
        <button onClick={() => changeMode('idle')} className="px-3 py-2 rounded bg-slate-400 text-white">Idle</button>
        <button onClick={handleCancelNav} disabled={!nav || (nav.status !== 1 && nav.status !== 2)} className="px-3 py-2 rounded bg-slate-200">Cancel Nav</button>
        <button onClick={handleEmergency} className="px-3 py-2 rounded bg-rose-600 text-white">Emergency Stop</button>
      </div>

      <div className="mt-4 text-sm text-blue-800 space-y-1">
        <div>Reported state: <span className="font-semibold">{robotState ?? '—'}</span></div>
        <div>Navigation: <span className="font-semibold">{nav?.text || '—'}</span></div>
      </div>
    </section>
  )
}

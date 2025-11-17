import { useEffect, useState } from 'react'
import { getModeStatus, startSlam, saveMapAndExitSlam, startLocalization, stopLocalization, stopSlam, reloadMaps, type ModeStatus } from '../lib/backend'

export default function ModePanel() {
  const [status, setStatus] = useState<ModeStatus>({ state: 'IDLE', slam_running: false, localization_running: false })
  const [busy, setBusy] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [scanSource, setScanSource] = useState<'raw' | 'filtered'>('raw') // placeholder for future param

  const refresh = async () => {
    try {
      const s = await getModeStatus(); setStatus(s)
    } catch (e: any) { setError(e?.message || 'Failed to get status') }
  }

  useEffect(() => { refresh() }, [])

  const doStartSlam = async () => {
    setBusy('Starting SLAM…'); setError(null)
    try { setStatus(await startSlam()) } catch (e: any) { setError(e?.message || 'Failed to start SLAM') } finally { setBusy(null) }
  }
  const doSaveExit = async () => {
    const name = (window.prompt('Map name (leave blank for timestamp):') || '').trim() || undefined
    setBusy('Saving map…'); setError(null)
    try {
      await saveMapAndExitSlam(name)
      await reloadMaps()
      await refresh()
      alert('Map saved and SLAM stopped')
    } catch (e: any) { setError(e?.message || 'Failed to save map') } finally { setBusy(null) }
  }
  const doStartLoc = async () => {
    setBusy('Starting Localization…'); setError(null)
    try { setStatus(await startLocalization()) } catch (e: any) { setError(e?.message || 'Failed to start Localization') } finally { setBusy(null) }
  }
  const doStopLoc = async () => {
    setBusy('Stopping Localization…'); setError(null)
    try { setStatus(await stopLocalization()) } catch (e: any) { setError(e?.message || 'Failed to stop Localization') } finally { setBusy(null) }
  }
  const doStopSlam = async () => {
    setBusy('Stopping SLAM…'); setError(null)
    try { setStatus(await stopSlam()) } catch (e: any) { setError(e?.message || 'Failed to stop SLAM') } finally { setBusy(null) }
  }
  const doReloadMaps = async () => { setBusy('Reloading maps…'); setError(null); try { await reloadMaps() } catch (e:any) { setError(e?.message||'Reload failed') } finally { setBusy(null) } }

  const slamActive = status.slam_running
  const locActive = status.localization_running

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Modes</h2>
      {error && (<div className="mb-2 p-2 text-sm text-red-700 bg-red-50 rounded border border-red-200">{error}</div>)}
      {busy && (<div className="mb-2 p-2 text-sm text-blue-700 bg-blue-50 rounded border border-blue-200">{busy}</div>)}

      <div className="grid grid-cols-2 gap-2">
        <button className="px-3 py-2 rounded bg-emerald-600 disabled:opacity-50 text-white" onClick={doStartSlam} disabled={slamActive || locActive}>Start SLAM Mode</button>
        <button className="px-3 py-2 rounded bg-amber-600 disabled:opacity-50 text-white" onClick={doSaveExit} disabled={!slamActive}>Save Map & Exit SLAM</button>
        <button className="px-3 py-2 rounded bg-slate-200" onClick={doReloadMaps}>Reload Map List</button>
        <div />
        <button className="px-3 py-2 rounded bg-indigo-600 disabled:opacity-50 text-white" onClick={doStartLoc} disabled={locActive || slamActive}>Start Localization Mode</button>
        <button className="px-3 py-2 rounded bg-rose-600 disabled:opacity-50 text-white" onClick={doStopLoc} disabled={!locActive}>Stop Localization Mode</button>
      </div>

      <div className="mt-4 text-sm text-blue-800">
        <div>Status: <span className="font-semibold">{status.state}</span></div>
        <div className="mt-2">Navigation status (placeholder): RUNNING — last error: none</div>
      </div>

      <div className="mt-3 text-sm text-blue-800">
        <div className="font-medium mb-1">Advanced</div>
        <label className="text-sm">Scan source:&nbsp;
          <select className="border rounded px-2 py-1" value={scanSource} onChange={(e)=>setScanSource(e.target.value as any)}>
            <option value="raw">raw</option>
            <option value="filtered">filtered</option>
          </select>
        </label>
      </div>
    </section>
  )
}


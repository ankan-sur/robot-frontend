import { DebugLog } from './DebugLog'

export function DebugPanel() {
  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      <h2 className="text-lg font-semibold mb-3 text-slate-800">Debug Log</h2>
      <DebugLog />
    </section>
  )
}

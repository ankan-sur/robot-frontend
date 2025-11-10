import { TeleopBlock } from './TeleopBlock'

export function DebugPanel() {
  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Debug Panel</h2>
      <div className="space-y-4">
        <div className="rounded-lg border-2 border-blue-300 bg-white p-4">
          <TeleopBlock />
        </div>
      </div>
    </section>
  )
}


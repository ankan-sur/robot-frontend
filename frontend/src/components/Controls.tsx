import { useRobotState } from '../ros/hooks'

type Props = {
  onStop: () => void
  disabledMove?: boolean
}

export function Controls({ onStop, disabledMove }: Props) {
  const robotState = useRobotState()
  const canIssueCommands = !(disabledMove)
  const stopEnabled = canIssueCommands && (robotState ? robotState !== 'idle' : true)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Controls</h2>
      <div className="space-y-4">
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

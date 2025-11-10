import { useState } from 'react'
import { changeMap, useAvailableMaps, publishInitialPose, DEMO_INITIAL_POSE } from '../ros/hooks'
import { TeleopBlock } from './TeleopBlock'
import { ROS_CONFIG } from '../ros/config'

type Props = {
  onMapChange?: (mapName: string) => void
}

export function DebugPanel({ onMapChange }: Props) {
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [loading, setLoading] = useState(false)
  const [status, setStatus] = useState<{ kind: 'success' | 'error'; text: string } | null>(null)
  const availableMaps = useAvailableMaps()

  const handleChangeMap = async () => {
    if (!selectedMap) return

    setLoading(true)
    setStatus(null)

    try {
      await changeMap(selectedMap)
      onMapChange?.(selectedMap)
      setStatus({ kind: 'success', text: `Map changed to ${selectedMap}` })
    } catch (err: any) {
      setStatus({ kind: 'error', text: err?.message || 'Failed to change map' })
      console.error('Map change error:', err)
    } finally {
      setLoading(false)
    }
  }

  const hasMaps = availableMaps.length > 0

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Debug Panel</h2>
      <div className="space-y-4">
        <div className="rounded-lg border-2 border-blue-300 bg-white p-4">
          <TeleopBlock />
        </div>
        <div className="rounded-lg border-2 border-blue-300 bg-white p-4">
          <div className="flex items-center justify-between">
            <div className="text-base font-medium text-blue-800">Localization</div>
            <button
              className="px-3 py-1.5 text-sm rounded bg-blue-600 text-white hover:bg-blue-500"
              onClick={async () => {
                try {
                  await publishInitialPose(DEMO_INITIAL_POSE.x, DEMO_INITIAL_POSE.y, DEMO_INITIAL_POSE.yaw)
                  setStatus({ kind: 'success', text: 'Initial pose published.' })
                } catch (e: any) {
                  setStatus({ kind: 'error', text: e?.message || 'Failed to publish initial pose' })
                }
              }}
            >
              Set Demo Initial Pose
            </button>
          </div>
          <div className="text-xs text-blue-700 mt-1">
            map → odom TF appears after setting an initial pose.
          </div>
        </div>
        {/* Map selection hidden for now */}
        <div className="text-sm text-blue-700 mt-2 space-y-2">
          <div>
            Maps from: <code className="bg-blue-100 px-2 py-0.5 rounded font-mono">ros2_ws/src/slam/maps</code>
          </div>
          <div className="flex flex-col gap-2">
            {ROS_CONFIG.rwt.rosboard && (
              <div className="flex items-center gap-2">
                <a
                  href={ROS_CONFIG.rwt.rosboard}
                  target="_blank"
                  rel="noreferrer"
                  className="px-3 py-1.5 text-sm rounded bg-indigo-600 text-white hover:bg-indigo-500"
                  title="Opens rosboard in a new tab"
                >
                  Open Rosboard (port 8888)
                </a>
              </div>
            )}
            <div className="text-xs text-blue-700">
              To start rosboard on the Pi:
              <pre className="mt-1 p-2 bg-blue-100 rounded text-[11px] whitespace-pre-wrap">
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "ros2 run rosboard rosboard_node --bind 0.0.0.0 --port 8888"</pre>
            </div>
          </div>
        </div>
        {status && (
          <div className={`text-sm rounded p-2 border ${status.kind === 'success' ? 'bg-green-50 text-green-700 border-green-200' : 'bg-red-50 text-red-700 border-red-200'}`}>
            {status.text}
          </div>
        )}
      </div>
    </section>
  )
}

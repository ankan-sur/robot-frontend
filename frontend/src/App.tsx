import { useState } from 'react'
import { MapView } from './components/MapView'
import { TeleopBlock } from './components/TeleopBlock'
import VideoFeed from './components/VideoFeed'
import { useRosConnection, useModeAndMaps, useRobotPose } from './ros/hooks'
import { setMode, loadMap, stopSlamAndSave } from './ros/services'

export default function App() {
  const { connected } = useRosConnection()
  const { mode, activeMap, maps, loading, refresh } = useModeAndMaps()
  const robotPose = useRobotPose()
  
  const [operating, setOperating] = useState(false)
  const [statusMsg, setStatusMsg] = useState<string | null>(null)
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [showCamera, setShowCamera] = useState(false)
  const [showSettings, setShowSettings] = useState(false)
  
  const clearStatus = () => setTimeout(() => setStatusMsg(null), 3000)

  const handleStartMapping = async () => {
    setOperating(true)
    setStatusMsg(null)
    try {
      await setMode('slam')
      setStatusMsg('✓ Mapping mode started')
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ ${e?.message || 'Failed'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleStopAndSave = async () => {
    const mapName = prompt('Enter map name:')
    if (!mapName?.trim()) {
      setStatusMsg('✗ Map name required')
      clearStatus()
      return
    }

    setOperating(true)
    setStatusMsg(null)
    try {
      await stopSlamAndSave(mapName.trim())
      setStatusMsg(`✓ Map "${mapName}" saved`)
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ ${e?.message || 'Failed'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleLoadMap = async () => {
    if (!selectedMap) {
      setStatusMsg('✗ Select a map first')
      clearStatus()
      return
    }

    setOperating(true)
    setStatusMsg(null)
    try {
      await loadMap(selectedMap)
      setStatusMsg(`✓ Loaded "${selectedMap}"`)
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ ${e?.message || 'Failed'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleSetMode = async (newMode: string) => {
    setOperating(true)
    setStatusMsg(null)
    try {
      await setMode(newMode)
      setStatusMsg(`✓ Switched to ${newMode}`)
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ ${e?.message || 'Failed'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  // Get robot position for display
  const pose = robotPose?.pose?.pose || robotPose?.pose
  const x = pose?.position?.x ?? 0
  const y = pose?.position?.y ?? 0
  const q = pose?.orientation
  const yaw = q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0

  return (
    <div className="min-h-screen bg-slate-900 text-white">
      {/* Mobile-optimized Header */}
      <div className="bg-slate-800 border-b border-slate-700 px-3 py-2">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-lg font-bold">HFH Robot</h1>
            <div className="flex items-center gap-2 text-xs">
              <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-400' : 'bg-red-400'}`} />
              <span className={connected ? 'text-green-400' : 'text-red-400'}>
                {connected ? 'Connected' : 'Disconnected'}
              </span>
              <span className="text-slate-400">•</span>
              <span className={`font-semibold ${
                mode === 'slam' ? 'text-amber-400' :
                mode === 'localization' ? 'text-blue-400' :
                'text-slate-400'
              }`}>
                {mode?.toUpperCase() || 'IDLE'}
              </span>
            </div>
          </div>
          <div className="flex gap-2">
            <button
              onClick={() => setShowCamera(!showCamera)}
              className={`px-3 py-1.5 rounded text-xs font-semibold transition-colors ${
                showCamera ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
              }`}
            >
              📷
            </button>
            <button
              onClick={() => setShowSettings(!showSettings)}
              className={`px-3 py-1.5 rounded text-xs font-semibold transition-colors ${
                showSettings ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
              }`}
            >
              ⚙️
            </button>
          </div>
        </div>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className={`mx-3 mt-2 p-2 rounded text-sm ${
          statusMsg.startsWith('✓') ? 'bg-green-900 text-green-200 border border-green-700' : 
          'bg-red-900 text-red-200 border border-red-700'
        }`}>
          {statusMsg}
        </div>
      )}

      <main className="p-3 space-y-3">
        {/* Map View */}
        <div className="bg-slate-800 rounded-lg overflow-hidden border border-slate-700">
          <MapView />
          {/* Robot Pose Overlay */}
          <div className="px-3 py-2 bg-slate-900/90 border-t border-slate-700">
            <div className="flex justify-between text-xs">
              <span className="text-slate-400">Position:</span>
              <span className="font-mono">
                x: {x.toFixed(2)}m, y: {y.toFixed(2)}m, θ: {(yaw * 180 / Math.PI).toFixed(0)}°
              </span>
            </div>
            {activeMap && (
              <div className="flex justify-between text-xs mt-1">
                <span className="text-slate-400">Map:</span>
                <span className="font-mono text-blue-400">{activeMap}</span>
              </div>
            )}
          </div>
        </div>

        {/* Camera Feed (collapsible) */}
        {showCamera && (
          <div className="bg-slate-800 rounded-lg overflow-hidden border border-slate-700">
            <VideoFeed />
          </div>
        )}

        {/* Mode Control */}
        <div className="bg-slate-800 rounded-lg border border-slate-700 p-3">
          <div className="text-sm font-semibold mb-2 text-slate-300">Mode Control</div>
          <div className="grid grid-cols-3 gap-2">
            <button
              onClick={() => handleSetMode('idle')}
              disabled={operating || mode === 'idle'}
              className="px-3 py-2 bg-slate-700 hover:bg-slate-600 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              Idle
            </button>
            <button
              onClick={handleStartMapping}
              disabled={operating || mode === 'slam'}
              className="px-3 py-2 bg-amber-600 hover:bg-amber-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              SLAM
            </button>
            <button
              onClick={() => handleSetMode('localization')}
              disabled={operating || mode === 'localization'}
              className="px-3 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              Localize
            </button>
          </div>

          {/* SLAM Save Button */}
          {mode === 'slam' && (
            <button
              onClick={handleStopAndSave}
              disabled={operating}
              className="w-full mt-2 px-4 py-2 bg-green-600 hover:bg-green-500 disabled:bg-slate-700 text-white rounded font-semibold transition-colors disabled:cursor-not-allowed"
            >
              💾 Stop & Save Map
            </button>
          )}

          {/* Map Loader */}
          {mode !== 'slam' && (
            <div className="mt-3 pt-3 border-t border-slate-700">
              <select
                value={selectedMap}
                onChange={(e) => setSelectedMap(e.target.value)}
                disabled={operating}
                className="w-full px-3 py-2 mb-2 bg-slate-700 text-white rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
              >
                <option value="">Select map...</option>
                {maps.map(m => (
                  <option key={m} value={m}>{m}</option>
                ))}
              </select>
              <button
                onClick={handleLoadMap}
                disabled={operating || !selectedMap}
                className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold transition-colors disabled:cursor-not-allowed"
              >
                Load Map
              </button>
            </div>
          )}
        </div>

        {/* Teleop Control */}
        <div className="bg-slate-800 rounded-lg border border-slate-700 p-3">
          <TeleopBlock />
        </div>

        {/* Settings Panel (collapsible) */}
        {showSettings && (
          <div className="bg-slate-800 rounded-lg border border-slate-700 p-3">
            <div className="text-sm font-semibold mb-2 text-slate-300">System Status</div>
            
            {/* Quick Status */}
            <div className="space-y-1 text-xs font-mono mb-3">
              <div className="flex justify-between">
                <span className="text-slate-400">ROS Connection:</span>
                <span className={connected ? 'text-green-400' : 'text-red-400'}>
                  {connected ? '● Connected' : '○ Disconnected'}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Current Mode:</span>
                <span className="text-white">{mode?.toUpperCase() || 'UNKNOWN'}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Active Map:</span>
                <span className="text-white">{activeMap || 'None'}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Maps Available:</span>
                <span className="text-white">{maps.length}</span>
              </div>
            </div>

            {/* Essential Topics */}
            <div className="mb-3 pb-3 border-t border-slate-700 pt-3">
              <div className="text-xs font-semibold text-slate-400 mb-2">Core Topics</div>
              <div className="space-y-1 text-xs font-mono text-slate-500">
                <div>/scan - LaserScan</div>
                <div>/odom - Odometry</div>
                <div>/imu/data - IMU</div>
                <div>/robot_pose - Pose</div>
                <div>/mode_manager/status - Mode</div>
                <div>/available_maps - Maps</div>
                <div>/ui/cmd_vel - Teleop</div>
                <div>/app/cmd_vel - Robot Control</div>
              </div>
            </div>

            {/* Services */}
            <div className="mb-3 pb-3 border-t border-slate-700 pt-3">
              <div className="text-xs font-semibold text-slate-400 mb-2">Services</div>
              <div className="space-y-1 text-xs font-mono text-slate-500">
                <div>/get_mode</div>
                <div>/set_mode</div>
                <div>/load_map</div>
                <div>/stop_slam_and_save</div>
              </div>
            </div>

            <button
              onClick={refresh}
              disabled={operating || loading}
              className="w-full px-4 py-2 bg-slate-700 hover:bg-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed disabled:opacity-50"
            >
              {loading ? '⟳ Refreshing...' : '↻ Refresh'}
            </button>
          </div>
        )}
      </main>
    </div>
  )
}

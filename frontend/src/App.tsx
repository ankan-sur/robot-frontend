import { useState, useEffect } from 'react'
import { MapView } from './components/MapView'
import { TeleopBlock } from './components/TeleopBlock'
import { DebugLog } from './components/DebugLog'
import VideoFeed from './components/VideoFeed'
import { useRosConnection, useModeAndMaps, useBattery, useRobotPose } from './ros/hooks'
import { setMode, stopSlamAndSave } from './ros/services'
import { isCloudModeEnabled } from './ros/ros'
import { useCloudStatus } from './ros/cloudHooks'

export default function App() {
  const { connected } = useRosConnection()
  const { mode, maps, loading, refresh } = useModeAndMaps()
  const battery = useBattery()
  const robotPose = useRobotPose()
  const cloudStatus = useCloudStatus()
  const isCloudMode = isCloudModeEnabled()
  
  const [operating, setOperating] = useState(false)
  const [statusMsg, setStatusMsg] = useState<string | null>(null)
  const [activeTab, setActiveTab] = useState<'map' | 'camera'>('map')
  const [showSaveDialog, setShowSaveDialog] = useState(false)
  const [saveMapName, setSaveMapName] = useState<string>('')
  
  const clearStatus = () => setTimeout(() => setStatusMsg(null), 5000)
  
  // Get robot position for telemetry display
  const pose = robotPose?.pose?.pose || robotPose?.pose
  const posX = pose?.position?.x ?? 0
  const posY = pose?.position?.y ?? 0
  const q = pose?.orientation
  const yaw = q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0

  // Auto-show map when in SLAM mode
  useEffect(() => {
    if (mode === 'slam') {
      setActiveTab('map')
    }
  }, [mode])

  const handleStartMapping = async () => {
    setOperating(true)
    setStatusMsg(null)
    try {
      await setMode('slam')
      setStatusMsg('✓ SLAM mode started - Drive around to map!')
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ Error: ${e?.message || 'Failed to start SLAM'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleCancelMapping = async () => {
    setOperating(true)
    setStatusMsg(null)
    try {
      await setMode('idle')
      setStatusMsg('✓ Mapping cancelled (map not saved)')
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ Error: ${e?.message || 'Failed to cancel'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleStopAndSave = async () => {
    setShowSaveDialog(true)
  }

  const confirmSaveMap = async () => {
    const mapName = saveMapName.trim()
    if (!mapName) {
      setStatusMsg('✗ Map name required')
      clearStatus()
      return
    }

    setShowSaveDialog(false)
    setSaveMapName('')
    setOperating(true)
    setStatusMsg('Saving map...')
    try {
      const result = await stopSlamAndSave(mapName)
      console.log('Save result:', result)
      setStatusMsg(`✓ Map "${mapName}" saved successfully!`)
      await refresh()
    } catch (e: any) {
      console.error('Save failed:', e)
      setStatusMsg(`✗ Error: ${e?.message || 'Failed to save map'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const cancelSaveDialog = () => {
    setShowSaveDialog(false)
    setSaveMapName('')
  }

  const isSlam = mode === 'slam'
  const isIdle = mode === 'idle' || !mode

  return (
    <div className="min-h-screen bg-slate-900 text-white">
      {/* Header */}
      <div className="bg-slate-800 border-b border-slate-700 px-4 py-3">
        <div className="max-w-7xl mx-auto">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-xl font-bold">HFH Robot - SLAM Mapping</h1>
              <div className="flex items-center gap-3 text-sm mt-1">
                <div className="flex items-center gap-1.5">
                  <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-400' : 'bg-red-400'}`} />
                  <span className={connected ? 'text-green-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Disconnected'}
                  </span>
                </div>
                <span className="text-slate-500">•</span>
                <span className={`font-semibold ${isSlam ? 'text-amber-400' : 'text-slate-400'}`}>
                  {isSlam ? '● MAPPING' : 'IDLE'}
                </span>
                {battery?.percent !== undefined && battery.percent !== null && (
                  <>
                    <span className="text-slate-500">•</span>
                    <span className={`${
                      (battery.percent ?? 0) < 20 ? 'text-red-400' :
                      (battery.percent ?? 0) < 40 ? 'text-yellow-400' :
                      'text-green-400'
                    }`}>
                      🔋 {Math.round(battery.percent ?? 0)}%
                    </span>
                  </>
                )}
              </div>
            </div>
            {isCloudMode && (
              <div className="text-right text-sm">
                <div className={cloudStatus.backendConnected ? 'text-green-400' : 'text-red-400'}>
                  Backend: {cloudStatus.backendConnected ? '●' : '○'}
                </div>
                <div className="text-slate-400">
                  Clients: {cloudStatus.uiClientCount}
                </div>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className="max-w-7xl mx-auto px-4 mt-3">
          <div className={`p-3 rounded-lg text-center font-semibold ${
            statusMsg.startsWith('✓') ? 'bg-green-900/50 text-green-300 border border-green-700' : 
            statusMsg.startsWith('✗') ? 'bg-red-900/50 text-red-300 border border-red-700' :
            'bg-blue-900/50 text-blue-300 border border-blue-700'
          }`}>
            {statusMsg}
          </div>
        </div>
      )}

      <main className="max-w-7xl mx-auto p-4">
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          
          {/* Left Column: Map/Camera + Debug Log */}
          <div className="lg:col-span-2 space-y-4">
            {/* Map/Camera Tabs */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 overflow-hidden">
              <div className="flex gap-2 px-4 py-2 border-b border-slate-700">
                <button
                  onClick={() => setActiveTab('map')}
                  className={`px-4 py-2 rounded font-medium transition-colors ${
                    activeTab === 'map' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
                  }`}
                >
                  Map
                </button>
                <button
                  onClick={() => setActiveTab('camera')}
                  className={`px-4 py-2 rounded font-medium transition-colors ${
                    activeTab === 'camera' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
                  }`}
                >
                  Camera
                </button>
              </div>
              <div className="p-4">
                {activeTab === 'map' ? (
                  <MapView embedded mode={mode} />
                ) : (
                  <VideoFeed embedded />
                )}
              </div>
            </div>

            {/* Debug Log - Always visible */}
            <DebugLog />
          </div>

          {/* Right Column: Teleop + Controls + Maps List + Telemetry */}
          <div className="space-y-4">
            {/* Teleop Control */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <TeleopBlock disableKeyboard={showSaveDialog} />
            </div>

            {/* SLAM Control Panel */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <h2 className="text-lg font-semibold mb-4 text-slate-200">SLAM Mapping Control</h2>
              
              {isIdle && (
                <div className="space-y-4">
                  <p className="text-slate-400 text-center">
                    Start SLAM to create a new map.
                  </p>
                  <button
                    onClick={handleStartMapping}
                    disabled={operating || !connected}
                    className="w-full px-6 py-4 bg-amber-600 hover:bg-amber-500 disabled:bg-slate-700 disabled:text-slate-500 text-white rounded-lg font-bold text-lg transition-colors"
                  >
                    {operating ? 'Starting...' : '▶ Start Mapping'}
                  </button>
                </div>
              )}

              {isSlam && (
                <div className="space-y-4">
                  <div className="bg-amber-900/30 border border-amber-700 rounded-lg p-3 text-center">
                    <div className="text-amber-400 font-semibold mb-1">● MAPPING</div>
                    <div className="text-slate-300 text-sm">Drive around to map</div>
                  </div>
                  
                  <div className="grid grid-cols-2 gap-2">
                    <button
                      onClick={handleCancelMapping}
                      disabled={operating}
                      className="px-3 py-2 bg-red-700 hover:bg-red-600 disabled:bg-slate-700 text-white rounded-lg font-semibold transition-colors"
                    >
                      ✗ Cancel
                    </button>
                    <button
                      onClick={handleStopAndSave}
                      disabled={operating}
                      className="px-3 py-2 bg-green-600 hover:bg-green-500 disabled:bg-slate-700 text-white rounded-lg font-semibold transition-colors"
                    >
                      ✓ Save Map
                    </button>
                  </div>
                </div>
              )}
            </div>

            {/* Saved Maps List */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <div className="flex items-center justify-between mb-3">
                <h2 className="text-base font-semibold text-slate-200">Saved Maps ({maps.length})</h2>
                <button
                  onClick={refresh}
                  disabled={loading}
                  className="px-2 py-1 text-xs bg-slate-700 hover:bg-slate-600 rounded transition-colors"
                >
                  {loading ? '...' : '↻'}
                </button>
              </div>
              {maps.length === 0 ? (
                <div className="text-slate-500 text-center py-3 text-sm">No maps saved yet</div>
              ) : (
                <div className="space-y-2 max-h-32 overflow-y-auto">
                  {maps.map((mapName) => (
                    <div
                      key={mapName}
                      className="flex items-center justify-between p-2 rounded bg-slate-700/50"
                    >
                      <span className="font-mono text-sm">{mapName}</span>
                    </div>
                  ))}
                </div>
              )}
            </div>

            {/* Telemetry Panel */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <div className="text-sm font-semibold mb-2 text-slate-300">Telemetry</div>
              <div className="space-y-1 text-xs font-mono">
                <div className="flex justify-between">
                  <span className="text-slate-400">Mode:</span>
                  <span className={isSlam ? 'text-amber-400' : 'text-slate-300'}>
                    {isSlam ? 'SLAM' : (mode || 'IDLE').toUpperCase()}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-400">ROS:</span>
                  <span className={connected ? 'text-green-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Offline'}
                  </span>
                </div>
                {isCloudMode && (
                  <>
                    <div className="flex justify-between">
                      <span className="text-slate-400">Backend:</span>
                      <span className={cloudStatus.backendConnected ? 'text-green-400' : 'text-red-400'}>
                        {cloudStatus.backendConnected ? 'Connected' : 'Offline'}
                      </span>
                    </div>
                  </>
                )}
                <div className="flex justify-between">
                  <span className="text-slate-400">Position:</span>
                  <span className="text-white">({posX.toFixed(2)}, {posY.toFixed(2)})</span>
                </div>
                {battery?.percent !== undefined && battery.percent !== null && (
                  <div className="flex justify-between">
                    <span className="text-slate-400">Battery:</span>
                    <span className={`${
                      (battery.percent ?? 0) < 20 ? 'text-red-400' :
                      (battery.percent ?? 0) < 40 ? 'text-yellow-400' :
                      'text-green-400'
                    }`}>
                      {Math.round(battery.percent ?? 0)}% {battery.volts ? `(${battery.volts.toFixed(1)}V)` : ''}
                    </span>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      </main>

      {/* Save Map Dialog */}
      {showSaveDialog && (
        <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-50 p-4">
          <div className="bg-slate-800 rounded-lg shadow-2xl max-w-md w-full p-6 border border-slate-600">
            <h2 className="text-xl font-bold text-white mb-2">Save Map</h2>
            <p className="text-slate-400 mb-4">Enter a name for your map:</p>
            <input
              type="text"
              value={saveMapName}
              onChange={(e) => setSaveMapName(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && saveMapName.trim()) confirmSaveMap()
                else if (e.key === 'Escape') cancelSaveDialog()
              }}
              placeholder="e.g., living_room, office_floor1"
              autoFocus
              className="w-full px-4 py-3 bg-slate-700 text-white border border-slate-600 rounded-lg focus:outline-none focus:border-blue-500 mb-4 text-lg"
            />
            <div className="flex gap-3">
              <button
                onClick={cancelSaveDialog}
                className="flex-1 px-4 py-3 bg-slate-700 hover:bg-slate-600 text-white rounded-lg font-semibold transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={confirmSaveMap}
                disabled={!saveMapName.trim()}
                className="flex-1 px-4 py-3 bg-green-600 hover:bg-green-500 disabled:bg-slate-700 disabled:text-slate-500 text-white rounded-lg font-semibold transition-colors"
              >
                Save Map
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}

import { useState, useEffect } from 'react'
import { MapView } from './components/MapView'
import { TeleopBlock } from './components/TeleopBlock'
import { DebugLog } from './components/DebugLog'
import VideoFeed from './components/VideoFeed'
import { useRosConnection, useModeAndMaps, useRobotPose, useBattery } from './ros/hooks'
import { setMode, loadMap, stopSlamAndSave, restartRobotStack } from './ros/services'

export default function App() {
  const { connected } = useRosConnection()
  const { mode, activeMap, maps, loading, refresh } = useModeAndMaps()
  const robotPose = useRobotPose()
  const battery = useBattery()
  // Disable POI system for now - not ready
  // const pois = usePoisForMap(activeMap || undefined)
  
  const [operating, setOperating] = useState(false)
  const [statusMsg, setStatusMsg] = useState<string | null>(null)
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [mapLoaded, setMapLoaded] = useState(false)
  const [activeTab, setActiveTab] = useState<'map' | 'camera'>('map')
  const [showSettings, setShowSettings] = useState(false)
  const [showDebugLog, setShowDebugLog] = useState(false)
  const [robotIp, setRobotIp] = useState<string>('')
  const [wifiSsid, setWifiSsid] = useState<string>('')
  const [showSaveDialog, setShowSaveDialog] = useState(false)
  const [saveMapName, setSaveMapName] = useState<string>('')
  
  const clearStatus = () => setTimeout(() => setStatusMsg(null), 3000)

  // Reset map selection when switching modes
  useEffect(() => {
    if (mode === 'idle') {
      setMapLoaded(false)
      setSelectedMap('')
    }
  }, [mode])

  // Auto-show camera when in idle mode
  useEffect(() => {
    if (mode === 'idle') {
      setActiveTab('camera')
    } else if (mode === 'slam' || mode === 'localization') {
      setActiveTab('map')
    }
  }, [mode])

  // Fetch network info from robot
  useEffect(() => {
    const fetchNetworkInfo = async () => {
      try {
        // Try to get IP from window.location or rosbridge connection
        const hostname = window.location.hostname
        if (hostname !== 'localhost' && hostname !== '127.0.0.1') {
          setRobotIp(hostname)
        }
        
        // TODO: Add ROS service call to get actual WiFi SSID from robot
        // For now, we'll leave it as a placeholder
        setWifiSsid('N/A')
      } catch (e) {
        console.error('Failed to fetch network info:', e)
      }
    }
    fetchNetworkInfo()
  }, [connected])

  const handleStartMapping = async () => {
    setOperating(true)
    setStatusMsg(null)
    try {
      await setMode('slam')
      setStatusMsg('[SUCCESS] Mapping mode started')
      refresh()
    } catch (e: any) {
      setStatusMsg(`[ERROR] ${e?.message || 'Failed'}`)
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
      setStatusMsg('[ERROR] Map name required')
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
      setStatusMsg(`[SUCCESS] Map "${mapName}" saved`)
      
      // After saving, switch to idle mode
      try {
        await setMode('idle')
      } catch (e: any) {
        console.warn('Failed to switch to idle after save:', e)
      }
      
      refresh()
    } catch (e: any) {
      console.error('Save failed:', e)
      setStatusMsg(`[ERROR] ${e?.message || 'Failed to save map'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const cancelSaveDialog = () => {
    setShowSaveDialog(false)
    setSaveMapName('')
  }

  const handleLoadMap = async () => {
    if (!selectedMap) {
      setStatusMsg('[ERROR] Select a map first')
      clearStatus()
      return
    }

    setOperating(true)
    setStatusMsg(null)
    try {
      await loadMap(selectedMap)
      setMapLoaded(true)
      setStatusMsg(`[SUCCESS] Loaded "${selectedMap}"`)
      refresh()
    } catch (e: any) {
      setStatusMsg(`[ERROR] ${e?.message || 'Failed'}`)
      setMapLoaded(false)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleSetMode = async (newMode: string) => {
    console.log(`[MODE SWITCH] Requesting mode change: ${mode} → ${newMode}`)
    setOperating(true)
    setStatusMsg(null)
    try {
      const result = await setMode(newMode)
      console.log(`[MODE SWITCH] Result:`, result)
      setStatusMsg(`[SUCCESS] Switched to ${newMode === 'localization' ? 'Nav' : newMode}`)
      refresh()
    } catch (e: any) {
      console.error(`[MODE SWITCH] Failed:`, e)
      setStatusMsg(`[ERROR] ${e?.message || 'Failed'}`)
    } finally {
      setOperating(false)
      clearStatus()
    }
  }

  const handleRestartRobotStack = async () => {
    const confirmed = window.confirm(
      'This will restart the entire ROS stack. The robot will be unavailable for ~30 seconds. Continue?'
    )
    if (!confirmed) return
    
    setOperating(true)
    setStatusMsg('Restarting robot stack...')
    try {
      await restartRobotStack()
      setStatusMsg('[SUCCESS] Robot stack restarting... Wait 30s and refresh page')
    } catch (e: any) {
      setStatusMsg(`[ERROR] Restart failed: ${e?.message || 'Unknown error'}`)
    } finally {
      setOperating(false)
      // Don't clear status - user needs to see the restart message
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
      {/* Responsive Header */}
      <div className="bg-slate-800 border-b border-slate-700 px-4 py-2">
        <div className="max-w-7xl mx-auto">
          <div className="flex items-center justify-between mb-2 sm:mb-0">
            <div>
              <div className="flex items-center gap-3 mb-1">
                <h1 className="text-xl font-bold">HFH Robot</h1>
                {/* Battery on mobile - compact display next to title */}
                {battery?.volts && (
                  <div className="flex sm:hidden items-center gap-1">
                    <span className="text-xs text-slate-400">BAT:</span>
                    <span className={`text-xs font-semibold ${
                      (battery?.percent ?? 0) < 20 ? 'text-red-400' :
                      (battery?.percent ?? 0) < 40 ? 'text-yellow-400' :
                      'text-green-400'
                    }`}>
                      {Math.round(battery?.percent ?? 0)}%
                    </span>
                  </div>
                )}
              </div>
              <div className="flex items-center gap-2 text-sm">
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
                  {mode === 'localization' ? 'NAV' : mode?.toUpperCase() || 'IDLE'}
                </span>
                {robotIp && (
                  <>
                    <span className="text-slate-400 hidden sm:inline">•</span>
                    <span className="text-slate-400 hidden sm:inline">{robotIp}</span>
                  </>
                )}
                {battery?.volts && (
                  <>
                    <span className="text-slate-400 hidden sm:inline">•</span>
                    <div className="hidden sm:flex items-center gap-2">
                      <span className="text-xs text-slate-400">BAT:</span>
                      <div className="w-20 h-3 bg-slate-700 rounded-full overflow-hidden">
                        <div 
                          className={`h-full transition-all ${
                            (battery?.percent ?? 0) < 20 ? 'bg-red-500' :
                            (battery?.percent ?? 0) < 40 ? 'bg-yellow-500' :
                            'bg-green-500'
                          }`}
                          style={{ width: `${Math.min(100, Math.max(0, battery?.percent ?? 0))}%` }}
                        />
                      </div>
                      <span className={`text-xs font-semibold ${
                        (battery?.percent ?? 0) < 20 ? 'text-red-400' :
                        (battery?.percent ?? 0) < 40 ? 'text-yellow-400' :
                        'text-green-400'
                      }`}>
                        {Math.round(battery?.percent ?? 0)}%
                      </span>
                      <span className="text-xs text-slate-500">
                        {battery.volts.toFixed(1)}V
                      </span>
                    </div>
                  </>
                )}
              </div>
            </div>
            <div className="hidden sm:flex gap-2">
              <button
                onClick={() => setShowDebugLog(!showDebugLog)}
                className={`px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
                  showDebugLog ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
                }`}
              >
                DEBUG Logs
              </button>
              <button
                onClick={() => setShowSettings(!showSettings)}
                className={`px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
                  showSettings ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
                }`}
              >
                SETTINGS
              </button>
            </div>
          </div>
          {/* Mobile-only button row */}
          <div className="flex gap-2 sm:hidden">
            <button
              onClick={() => setShowDebugLog(!showDebugLog)}
              className={`flex-1 px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
                showDebugLog ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
              }`}
            >
              DEBUG Logs
            </button>
            <button
              onClick={() => setShowSettings(!showSettings)}
              className={`flex-1 px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
                showSettings ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
              }`}
            >
              SETTINGS
            </button>
          </div>
        </div>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className="max-w-7xl mx-auto px-4">
          <div className={`mt-2 p-2 rounded text-base ${
            statusMsg.startsWith('[SUCCESS]') ? 'bg-green-900 text-green-200 border border-green-700' : 
            'bg-red-900 text-red-200 border border-red-700'
          }`}>
            {statusMsg}
          </div>
        </div>
      )}

      <main className="max-w-7xl mx-auto p-4">
        {/* Responsive Grid: Stack on mobile, side-by-side on desktop */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          {/* Left Column: Map/Camera + Mode Control + Debug Logs */}
          <div className="lg:col-span-2 space-y-4 order-1 lg:order-1">
            {/* Map/Camera Tabs */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 overflow-hidden">
              {/* Tab Headers */}
              <div className="flex items-center justify-between px-4 py-2 border-b border-slate-700">
                <div className="flex gap-2">
                  <button
                    onClick={() => setActiveTab('map')}
                    className={`px-4 py-2 rounded text-base font-medium transition-colors ${
                      activeTab === 'map' 
                        ? 'bg-blue-600 text-white' 
                        : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
                    }`}
                  >
                    Map
                  </button>
                  <button
                    onClick={() => setActiveTab('camera')}
                    className={`px-4 py-2 rounded text-base font-medium transition-colors ${
                      activeTab === 'camera' 
                        ? 'bg-blue-600 text-white' 
                        : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
                    }`}
                  >
                    Camera
                  </button>
                </div>
              </div>

              {/* Tab Content */}
              <div className="p-4">
                {activeTab === 'map' ? (
                  <MapView embedded mode={mode} />
                ) : (
                  <VideoFeed embedded />
                )}
              </div>

              {/* Robot Pose Overlay (only show on map tab) */}
              {activeTab === 'map' && (
                <div className="px-4 py-2 bg-slate-900/90 border-t border-slate-700">
                  <div className="flex justify-between text-sm">
                    <span className="text-slate-400">Position:</span>
                    <span className="font-mono">
                      x: {x.toFixed(2)}m, y: {y.toFixed(2)}m, θ: {(yaw * 180 / Math.PI).toFixed(0)}°
                    </span>
                  </div>
                  {activeMap && (
                    <div className="flex justify-between text-sm mt-1">
                      <span className="text-slate-400">Map:</span>
                      <span className="font-mono text-blue-400">{activeMap}</span>
                    </div>
                  )}
                </div>
              )}
            </div>

            {/* Mode Control - Compact layout */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4 lg:order-2 order-3">
              <div className="text-base font-semibold mb-3 text-slate-300">Mode Control</div>
              <div className="grid grid-cols-3 gap-2 mb-3">
                <button
                  onClick={() => handleSetMode('idle')}
                  disabled={operating || mode === 'idle'}
                  className="px-2 py-2 bg-slate-700 hover:bg-slate-600 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                >
                  Idle
                </button>
                <button
                  onClick={handleStartMapping}
                  disabled={operating || mode === 'slam'}
                  className="px-2 py-2 bg-amber-600 hover:bg-amber-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                >
                  SLAM
                </button>
                <button
                  onClick={() => handleSetMode('localization')}
                  disabled={operating || mode === 'localization'}
                  className="px-2 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                >
                  Nav
                </button>
              </div>

              {/* SLAM Mode: Just save map, that's it */}
              {mode === 'slam' && (
                <div className="space-y-2">
                  <div className="text-sm text-slate-400 text-center py-2">
                    Drive around to map the area
                  </div>
                  <button
                    onClick={handleStopAndSave}
                    disabled={operating}
                    className="w-full px-4 py-3 bg-green-600 hover:bg-green-500 disabled:bg-slate-700 text-white rounded font-bold text-base transition-colors disabled:cursor-not-allowed"
                  >
                    Save Map & Stop
                  </button>
                </div>
              )}

              {/* Nav Mode (Localization): Map selection only */}
              {mode === 'localization' && (
                <div className="space-y-2">
                  <div className="text-xs text-slate-400 mb-1">Select Map</div>
                  <select
                    value={selectedMap}
                    onChange={(e) => {
                      setSelectedMap(e.target.value)
                      setMapLoaded(false)
                    }}
                    disabled={operating || mapLoaded}
                    className="w-full px-3 py-2 bg-slate-700 text-white text-sm rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
                  >
                    <option value="">Select map...</option>
                    {maps.map(m => (
                      <option key={m} value={m}>{m}</option>
                    ))}
                  </select>
                  
                  <button
                    onClick={handleLoadMap}
                    disabled={operating || !selectedMap || mapLoaded}
                    className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                  >
                    {mapLoaded ? '[SUCCESS] Map Loaded' : 'Load Map'}
                  </button>
                </div>
              )}

              {/* Idle Mode: View-only map display */}
              {mode === 'idle' && (
                <div className="space-y-2">
                  <div className="text-xs text-slate-400 mb-1">View Saved Map</div>
                  <select
                    value={selectedMap}
                    onChange={async (e) => {
                      const mapName = e.target.value
                      setSelectedMap(mapName)
                      if (mapName) {
                        // Just load the map image for viewing, don't start localization
                        try {
                          console.log('[IDLE] Loading map image:', mapName)
                          await loadMap(mapName)
                          console.log('[IDLE] Map loaded successfully')
                        } catch (err: any) {
                          console.error('[IDLE] Map load failed:', err)
                          setStatusMsg(`[ERROR] Failed to load map: ${err.message}`)
                          setTimeout(() => setStatusMsg(null), 3000)
                        }
                      }
                    }}
                    disabled={operating}
                    className="w-full px-3 py-2 bg-slate-700 text-white text-sm rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
                  >
                    <option value="">Select map to view...</option>
                    {maps.map(m => (
                      <option key={m} value={m}>{m}</option>
                    ))}
                  </select>
                  <div className="text-xs text-slate-500 text-center">
                    View only - Switch to Nav mode to navigate
                  </div>
                </div>
              )}
            </div>

            {/* Debug Logs (collapsible) */}
            {showDebugLog && (
              <div className="lg:order-3 order-4">
                <DebugLog />
              </div>
            )}
          </div>

          {/* Right Column: Teleop + Settings - Appears 2nd on mobile, right side on desktop */}
          <div className="space-y-4 order-2 lg:order-2">
            {/* Teleop Control */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <TeleopBlock />
            </div>

            {/* Settings Panel (collapsible) */}
            {showSettings && (
              <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
                <div className="text-base font-semibold mb-3 text-slate-300">System Status</div>
                
                {/* Network Info */}
                <div className="space-y-2 text-sm font-mono mb-3 pb-3 border-b border-slate-700">
                  <div className="flex justify-between">
                    <span className="text-slate-400">Robot IP:</span>
                    <span className="text-white">{robotIp || 'Unknown'}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-slate-400">WiFi:</span>
                    <span className="text-white">{wifiSsid || 'N/A'}</span>
                  </div>
                </div>

                {/* Quick Status */}
                <div className="space-y-2 text-sm font-mono mb-3 pb-3 border-b border-slate-700">
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

                <button
                  onClick={refresh}
                  disabled={operating || loading}
                  className="w-full px-4 py-2.5 bg-slate-700 hover:bg-slate-600 text-white rounded font-semibold text-base transition-colors disabled:cursor-not-allowed disabled:opacity-50"
                >
                  {loading ? '⟳ Refreshing...' : '↻ Refresh'}
                </button>

                {/* Emergency Restart Button */}
                <button
                  onClick={handleRestartRobotStack}
                  disabled={operating}
                  className="w-full px-4 py-2.5 bg-red-700 hover:bg-red-600 disabled:bg-slate-700 text-white rounded font-semibold text-base transition-colors disabled:cursor-not-allowed disabled:opacity-50 mt-2"
                >
                  🔄 Restart Robot Stack
                </button>
                <div className="text-xs text-slate-500 text-center mt-1">
                  Use if modes are stuck or system unresponsive
                </div>
              </div>
            )}
          </div>
        </div>
      </main>

      {/* Save Map Dialog Modal */}
      {showSaveDialog && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4">
          <div className="bg-slate-800 rounded-lg shadow-xl max-w-md w-full p-6">
            <h2 className="text-xl font-bold text-white mb-4">Save Map</h2>
            <p className="text-slate-300 mb-4">Enter a name for the map:</p>
            <input
              type="text"
              value={saveMapName}
              onChange={(e) => setSaveMapName(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && saveMapName.trim()) {
                  confirmSaveMap()
                } else if (e.key === 'Escape') {
                  cancelSaveDialog()
                }
              }}
              placeholder="map_name"
              autoFocus
              className="w-full px-4 py-2 bg-slate-700 text-white border border-slate-600 rounded focus:outline-none focus:border-blue-500 mb-4"
            />
            <div className="flex gap-3">
              <button
                onClick={cancelSaveDialog}
                className="flex-1 px-4 py-2 bg-slate-700 hover:bg-slate-600 text-white rounded font-semibold transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={confirmSaveMap}
                disabled={!saveMapName.trim()}
                className="flex-1 px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-700 text-white rounded font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-50"
              >
                Save
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}

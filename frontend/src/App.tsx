import { useState, useEffect } from 'react'
import { MapView } from './components/MapView'
import { TeleopBlock } from './components/TeleopBlock'
import { DebugLog } from './components/DebugLog'
import VideoFeed from './components/VideoFeed'
import { useRosConnection, useModeAndMaps, useRobotPose, useBattery, usePoisForMap } from './ros/hooks'
import { setMode, loadMap, stopSlamAndSave, markPOI } from './ros/services'

export default function App() {
  const { connected } = useRosConnection()
  const { mode, activeMap, maps, loading, refresh } = useModeAndMaps()
  const robotPose = useRobotPose()
  const battery = useBattery()
  const pois = usePoisForMap(activeMap || undefined)
  
  const [operating, setOperating] = useState(false)
  const [statusMsg, setStatusMsg] = useState<string | null>(null)
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [selectedPoi, setSelectedPoi] = useState<string>('')
  const [activeTab, setActiveTab] = useState<'map' | 'camera'>('map')
  const [showSettings, setShowSettings] = useState(false)
  const [showDebugLog, setShowDebugLog] = useState(false)
  const [robotIp, setRobotIp] = useState<string>('')
  const [wifiSsid, setWifiSsid] = useState<string>('')
  
  const clearStatus = () => setTimeout(() => setStatusMsg(null), 3000)

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

  const handleMarkPOI = async () => {
    const poiName = prompt('Enter POI name:')
    if (!poiName?.trim()) {
      setStatusMsg('✗ POI name required')
      clearStatus()
      return
    }

    setOperating(true)
    setStatusMsg(null)
    try {
      // markPOI will use current robot pose if pose not provided
      await markPOI({ name: poiName.trim() })
      setStatusMsg(`✓ POI "${poiName}" marked`)
      refresh()
    } catch (e: any) {
      setStatusMsg(`✗ ${e?.message || 'Failed to mark POI'}`)
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
      {/* Responsive Header */}
      <div className="bg-slate-800 border-b border-slate-700 px-4 py-2">
        <div className="max-w-7xl mx-auto flex items-center justify-between">
          <div>
            <h1 className="text-xl font-bold">HFH Robot</h1>
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
                {mode?.toUpperCase() || 'IDLE'}
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
                  <span className={`hidden sm:inline font-semibold ${
                    (battery?.percent ?? 0) < 20 ? 'text-red-400' :
                    (battery?.percent ?? 0) < 40 ? 'text-yellow-400' :
                    'text-green-400'
                  }`}>
                    🔋 {battery.volts.toFixed(1)}V ({Math.round(battery?.percent ?? 0)}%)
                  </span>
                </>
              )}
            </div>
          </div>
          <div className="flex gap-2">
            <button
              onClick={() => setShowDebugLog(!showDebugLog)}
              className={`px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
                showDebugLog ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-300'
              }`}
            >
              🐛 Logs
            </button>
            <button
              onClick={() => setShowSettings(!showSettings)}
              className={`px-3 py-1.5 rounded text-sm font-semibold transition-colors ${
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
        <div className="max-w-7xl mx-auto px-4">
          <div className={`mt-2 p-2 rounded text-base ${
            statusMsg.startsWith('✓') ? 'bg-green-900 text-green-200 border border-green-700' : 
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
                    🗺️ Map
                  </button>
                  <button
                    onClick={() => setActiveTab('camera')}
                    className={`px-4 py-2 rounded text-base font-medium transition-colors ${
                      activeTab === 'camera' 
                        ? 'bg-blue-600 text-white' 
                        : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
                    }`}
                  >
                    📷 Camera
                  </button>
                </div>
              </div>

              {/* Tab Content */}
              <div className="p-4">
                {activeTab === 'map' ? (
                  <MapView embedded />
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
                  Localize
                </button>
              </div>

              {/* SLAM Mode: Mark POI + Save Map */}
              {mode === 'slam' && (
                <div className="space-y-2">
                  <button
                    onClick={handleMarkPOI}
                    disabled={operating}
                    className="w-full px-4 py-2 bg-purple-600 hover:bg-purple-500 disabled:bg-slate-700 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                  >
                    📍 Mark POI
                  </button>
                  <button
                    onClick={handleStopAndSave}
                    disabled={operating}
                    className="w-full px-4 py-2 bg-green-600 hover:bg-green-500 disabled:bg-slate-700 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                  >
                    💾 Stop & Save Map
                  </button>
                </div>
              )}

              {/* Localization Mode: Map + POI selection */}
              {mode === 'localization' && (
                <div className="space-y-2">
                  <select
                    value={selectedMap}
                    onChange={(e) => setSelectedMap(e.target.value)}
                    disabled={operating}
                    className="w-full px-3 py-2 bg-slate-700 text-white text-sm rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
                  >
                    <option value="">Select map...</option>
                    {maps.map(m => (
                      <option key={m} value={m}>{m}</option>
                    ))}
                  </select>
                  <button
                    onClick={handleLoadMap}
                    disabled={operating || !selectedMap}
                    className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                  >
                    Load Map
                  </button>
                  
                  {/* POI Dropdown - only show after map loaded */}
                  {activeMap && (
                    <>
                      <select
                        value={selectedPoi}
                        onChange={(e) => setSelectedPoi(e.target.value)}
                        disabled={operating || pois.length === 0}
                        className="w-full px-3 py-2 bg-slate-700 text-white text-sm rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
                      >
                        <option value="">Select POI...</option>
                        {pois.map(poi => (
                          <option key={poi.name} value={poi.name}>{poi.name}</option>
                        ))}
                      </select>
                      {pois.length === 0 && (
                        <div className="text-xs text-slate-500 text-center">No POIs for this map</div>
                      )}
                    </>
                  )}
                </div>
              )}

              {/* Idle Mode: Show map loader */}
              {mode === 'idle' && maps.length > 0 && (
                <div className="pt-3 border-t border-slate-700">
                  <select
                    value={selectedMap}
                    onChange={(e) => setSelectedMap(e.target.value)}
                    disabled={operating}
                    className="w-full px-3 py-2 mb-2 bg-slate-700 text-white text-sm rounded border border-slate-600 focus:border-blue-500 focus:outline-none disabled:bg-slate-900 disabled:text-slate-600"
                  >
                    <option value="">Select map...</option>
                    {maps.map(m => (
                      <option key={m} value={m}>{m}</option>
                    ))}
                  </select>
                  <button
                    onClick={handleLoadMap}
                    disabled={operating || !selectedMap}
                    className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-900 disabled:text-slate-600 text-white rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
                  >
                    Load Map
                  </button>
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

                {/* Essential Topics */}
                <div className="mb-3 pb-3 border-t border-slate-700 pt-3">
                  <div className="text-sm font-semibold text-slate-400 mb-2">Core Topics</div>
                  <div className="space-y-1 text-sm font-mono text-slate-500">
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
                  <div className="text-sm font-semibold text-slate-400 mb-2">Services</div>
                  <div className="space-y-1 text-sm font-mono text-slate-500">
                    <div>/get_mode</div>
                    <div>/set_mode</div>
                    <div>/load_map</div>
                    <div>/stop_slam_and_save</div>
                  </div>
                </div>

                <button
                  onClick={refresh}
                  disabled={operating || loading}
                  className="w-full px-4 py-2.5 bg-slate-700 hover:bg-slate-600 text-white rounded font-semibold text-base transition-colors disabled:cursor-not-allowed disabled:opacity-50"
                >
                  {loading ? '⟳ Refreshing...' : '↻ Refresh'}
                </button>
              </div>
            )}
          </div>
        </div>
      </main>
    </div>
  )
}

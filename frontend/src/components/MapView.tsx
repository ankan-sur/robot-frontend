import { useEffect, useRef, useMemo, useState } from 'react'
import { useMap, useOdom, useAvailableMaps, changeMap, usePointsOfInterest, useNavigateToPose } from '../ros/hooks'

export function MapView() {
  const map = useMap()
  const odom = useOdom()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const maps = useAvailableMaps()
  const pois = usePointsOfInterest()
  const { navigate } = useNavigateToPose()

  const [selectedMap, setSelectedMap] = useState<string>('')
  const [selectedPoi, setSelectedPoi] = useState<string>('')
  const [navState, setNavState] = useState<'idle' | 'sending' | 'success' | 'error'>('idle')
  const [navMsg, setNavMsg] = useState<string>('')
  const mapReady = Boolean(map)

  // Keep selected map stable if list refreshes
  useEffect(() => {
    if (!selectedMap && maps.length > 0) {
      // Do not auto-switch maps; wait for user selection
      // setSelectedMap(maps[0])
    }
  }, [maps, selectedMap])

  const poiOptions = useMemo(() => pois.map(p => p.name || `${p.x.toFixed(2)},${p.y.toFixed(2)}`), [pois])

  const handleMapChange = async (name: string) => {
    setSelectedMap(name)
    setSelectedPoi('')
    try {
      if (name) await changeMap(name)
    } catch (e) {
      console.error('Failed to change map:', e)
    }
  }

  const handleNavigate = async () => {
    if (!selectedPoi) return
    // Find POI by name; if not found, parse "x,y[,yaw]"
    let x: number | null = null
    let y: number | null = null
    let yaw = 0
    const byName = pois.find(p => (p.name || '') === selectedPoi)
    if (byName) {
      x = byName.x; y = byName.y; yaw = byName.yaw ?? 0
    } else {
      const parts = selectedPoi.split(',').map(s => parseFloat(s.trim()))
      if (parts.length >= 2 && parts.every(n => !isNaN(n))) {
        x = parts[0]; y = parts[1]; if (!isNaN(parts[2])) yaw = parts[2]
      }
    }
    if (x == null || y == null) return
    try {
      setNavState('sending'); setNavMsg('Sending goal…')
      await navigate(x, y, yaw)
      setNavState('success'); setNavMsg('Arrived at goal.')
      setTimeout(() => { setNavState('idle'); setNavMsg('') }, 3000)
    } catch (e: any) {
      setNavState('error'); setNavMsg(e?.message || 'Failed to navigate')
    }
  }

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || !map) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const { width, height, resolution, origin } = map.info
    const data = map.data
    canvas.width = width
    canvas.height = height
    const imageData = ctx.createImageData(width, height)
    for (let i = 0; i < data.length; i++) {
      const v = data[i]
      const idx = i * 4
      if (v === -1) {
        imageData.data[idx] = 128; imageData.data[idx+1]=128; imageData.data[idx+2]=128; imageData.data[idx+3]=255
      } else if (v === 0) {
        imageData.data[idx] = 255; imageData.data[idx+1]=255; imageData.data[idx+2]=255; imageData.data[idx+3]=255
      } else {
        imageData.data[idx] = 0; imageData.data[idx+1]=0; imageData.data[idx+2]=0; imageData.data[idx+3]=255
      }
    }
    ctx.putImageData(imageData, 0, 0)

    // draw robot
    const pos = odom?.pose.pose.position
    const ori = odom?.pose.pose.orientation
    if (pos && ori) {
      const mapX = (pos.x - origin.position.x) / resolution
      const mapY = (pos.y - origin.position.y) / resolution
      const canvasY = height - mapY
      const yaw = 2 * Math.atan2(ori.z, ori.w)
      ctx.save()
      ctx.translate(mapX, canvasY)
      ctx.rotate(yaw)
      ctx.fillStyle = '#2563eb'
      ctx.beginPath(); ctx.moveTo(0,-8); ctx.lineTo(-5,5); ctx.lineTo(5,5); ctx.closePath(); ctx.fill()
      ctx.restore()
    }

    // draw POIs
    if (pois && Array.isArray(pois)) {
      ctx.save()
      ctx.font = '10px sans-serif'
      ctx.textAlign = 'left'
      ctx.textBaseline = 'middle'
      pois.forEach(p => {
        const px = (p.x - origin.position.x) / resolution
        const py = (p.y - origin.position.y) / resolution
        const cy = height - py
        const isSelected = selectedPoi && (p.name === selectedPoi)
        ctx.fillStyle = isSelected ? '#22c55e' : '#16a34a'
        ctx.strokeStyle = '#064e3b'
        ctx.beginPath()
        ctx.arc(px, cy, 3, 0, Math.PI * 2)
        ctx.fill()
        ctx.stroke()
        const label = p.name || `${p.x.toFixed(2)}, ${p.y.toFixed(2)}`
        ctx.fillStyle = '#064e3b'
        ctx.fillText(label, px + 5, cy)
      })
      ctx.restore()
    }
  }, [map, odom, pois, selectedPoi])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-4 gap-3">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map</h2>
        <div className="flex items-center gap-2">
          {/* Map selector */}
          <label className="text-sm text-blue-800">Map</label>
          <select
            className="text-sm border border-blue-300 rounded px-2 py-1 bg-white text-blue-900"
            value={selectedMap}
            onChange={(e) => handleMapChange(e.target.value)}
          >
            <option value="">Select map…</option>
            {maps.map((m) => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>

          {/* POI selector */}
          <label className="text-sm text-blue-800 ml-3">POI</label>
          <select
            className="text-sm border border-blue-300 rounded px-2 py-1 bg-white text-blue-900 disabled:opacity-50"
            value={selectedPoi}
            onChange={(e) => setSelectedPoi(e.target.value)}
            disabled={!mapReady || !selectedMap}
          >
            <option value="">Select POI…</option>
            {poiOptions.map((name) => (
              <option key={name} value={name}>{name}</option>
            ))}
          </select>
        </div>
      </div>
      <div className="h-96 bg-gradient-to-br from-blue-50 to-indigo-50 rounded-lg border-2 border-blue-300 overflow-hidden relative">
        <canvas ref={canvasRef} className="w-full h-full object-contain" style={{ imageRendering: 'pixelated' }} />
      </div>
      <div className="flex items-center justify-end mt-3 gap-2">
        {navMsg && (
          <span className={`text-sm ${navState === 'error' ? 'text-red-700' : navState === 'success' ? 'text-green-700' : 'text-blue-700'}`}>{navMsg}</span>
        )}
        <button
          className="px-3 py-1.5 text-sm rounded bg-blue-600 text-white disabled:opacity-50 disabled:cursor-not-allowed hover:bg-blue-500"
          onClick={handleNavigate}
          disabled={!selectedPoi || navState === 'sending'}
          title={!selectedPoi ? 'Select a POI first' : 'Send navigation goal'}
        >
          {navState === 'sending' ? 'Navigating…' : 'Navigate'}
        </button>
      </div>
    </section>
  )
}

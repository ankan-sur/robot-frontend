import { useEffect, useRef, useState } from 'react'
import { useMap, useRobotPose, usePointsOfInterest } from '../ros/hooks'
import { markPOI } from '../ros/services'

type Props = {
  embedded?: boolean // when true, render without outer section/header for tab embedding
}

export function MapView({ embedded = false }: Props) {
  const map = useMap()
  const robotPose = useRobotPose() // subscribe to /robot/pose (map frame)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const pois = usePointsOfInterest()
  const mapReady = Boolean(map)
  const [addPoiMode, setAddPoiMode] = useState(false)
  const [statusMsg, setStatusMsg] = useState<string | null>(null)

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

    // draw robot using /robot/pose (map frame)
    const pos = robotPose?.pose?.position
    const ori = robotPose?.pose?.orientation
    if (pos && ori) {
      const mapX = (pos.x - origin.position.x) / resolution
      const mapY = (pos.y - origin.position.y) / resolution
      const canvasY = height - mapY
      // yaw from quaternion
      const yaw = 2 * Math.atan2(ori.z, ori.w)
      ctx.save()
      ctx.translate(mapX, canvasY)
      ctx.rotate(yaw)
      ctx.fillStyle = '#2563eb'
      ctx.beginPath(); ctx.moveTo(0,-8); ctx.lineTo(-5,5); ctx.lineTo(5,5); ctx.closePath(); ctx.fill()
      ctx.restore()
    }

    // draw POIs (support legacy and new schema)
    if (pois && Array.isArray(pois)) {
      ctx.save()
      ctx.font = '10px sans-serif'
      ctx.textAlign = 'left'
      ctx.textBaseline = 'middle'
      pois.forEach(p => {
        let px: number | null = null
        let py: number | null = null
        if (p.pose && typeof p.pose.x === 'number' && typeof p.pose.y === 'number') {
          px = (p.pose.x - origin.position.x) / resolution
          py = (p.pose.y - origin.position.y) / resolution
        } else if (typeof p.x === 'number' && typeof p.y === 'number') {
          px = (p.x - origin.position.x) / resolution
          py = (p.y - origin.position.y) / resolution
        }
        if (px == null || py == null) return
        const cy = height - py
        ctx.fillStyle = '#16a34a'
        ctx.strokeStyle = '#064e3b'
        ctx.beginPath()
        ctx.arc(px, cy, 3, 0, Math.PI * 2)
        ctx.fill()
        ctx.stroke()
        const label = p.name || (p.pose ? `${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)}` : `${(p.x as number).toFixed(2)}, ${(p.y as number).toFixed(2)}`)
        ctx.fillStyle = '#064e3b'
        ctx.fillText(label, px + 5, cy)
      })
      ctx.restore()
    }
  }, [map, robotPose, pois])

  const handleCanvasClick = async (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!addPoiMode || !map || !canvasRef.current) return

    const canvas = canvasRef.current
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    const canvasX = (e.clientX - rect.left) * scaleX
    const canvasY = (e.clientY - rect.top) * scaleY

    // Convert canvas coordinates to map coordinates
    const { width, height, resolution, origin } = map.info
    const mapY = height - canvasY
    const mapX = canvasX * resolution + origin.position.x
    const mapYCoord = mapY * resolution + origin.position.y

    const name = prompt('Enter POI name:')
    if (!name || !name.trim()) {
      setStatusMsg('✗ POI name required')
      setTimeout(() => setStatusMsg(null), 3000)
      return
    }

    try {
      await markPOI({
        name: name.trim(),
        pose: { x: mapX, y: mapYCoord, yaw: 0 }
      })
      setStatusMsg(`✓ Added POI "${name}"`)
      setTimeout(() => setStatusMsg(null), 3000)
      setAddPoiMode(false)
    } catch (err: any) {
      setStatusMsg(`✗ Failed: ${err?.message || 'Unknown error'}`)
      setTimeout(() => setStatusMsg(null), 3000)
    }
  }

  const content = (
    <div className="relative">
      {statusMsg && (
        <div className={`absolute top-2 left-1/2 transform -translate-x-1/2 z-10 px-3 py-2 rounded shadow-lg text-sm ${
          statusMsg.startsWith('✓') ? 'bg-green-600 text-white' : 'bg-red-600 text-white'
        }`}>
          {statusMsg}
        </div>
      )}
      <div className="h-96 bg-slate-900 rounded overflow-hidden relative flex items-center justify-center">
        {!map && (
          <div className="absolute inset-0 flex items-center justify-center text-center p-4 text-slate-400">
            <div>
              <div className="font-semibold mb-1">No map available</div>
              <div className="text-sm">Start SLAM or Localization to display the map.</div>
            </div>
          </div>
        )}
        <canvas 
          ref={canvasRef} 
          className={`w-full h-full object-contain ${addPoiMode ? 'cursor-crosshair' : ''}`}
          style={{ imageRendering: 'pixelated' }}
          onClick={handleCanvasClick}
        />
      </div>
      {map && (
        <div className="mt-2 flex items-center justify-between">
          <button
            onClick={() => setAddPoiMode(!addPoiMode)}
            className={`px-3 py-1 rounded text-sm font-medium ${
              addPoiMode 
                ? 'bg-orange-600 text-white hover:bg-orange-700' 
                : 'bg-blue-600 text-white hover:bg-blue-700'
            }`}
          >
            {addPoiMode ? '✕ Cancel Add POI' : '📍 Click to Add POI'}
          </button>
          {addPoiMode && (
            <span className="text-xs text-slate-600">Click on the map to place a POI</span>
          )}
        </div>
      )}
    </div>
  )

  if (embedded) return content

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-lg font-semibold text-slate-800">Map</h2>
      </div>
      {content}
    </section>
  )
}

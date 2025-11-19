import { useEffect, useRef } from 'react'
import { useMap, useRobotPose, usePointsOfInterest } from '../ros/hooks'

export function MapView() {
  const map = useMap()
  const robotPose = useRobotPose() // subscribe to /robot/pose (map frame)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const pois = usePointsOfInterest()
  const mapReady = Boolean(map)

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
        let px, py, yaw
        if (p.pose) {
          px = (p.pose.x - origin.position.x) / resolution
          py = (p.pose.y - origin.position.y) / resolution
          yaw = p.pose.yaw || 0
        } else {
          px = (p.x - origin.position.x) / resolution
          py = (p.y - origin.position.y) / resolution
          yaw = 0
        }
        const cy = height - py
        ctx.fillStyle = '#16a34a'
        ctx.strokeStyle = '#064e3b'
        ctx.beginPath()
        ctx.arc(px, cy, 3, 0, Math.PI * 2)
        ctx.fill()
        ctx.stroke()
        const label = p.name || (p.pose ? `${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)}` : `${p.x.toFixed(2)}, ${p.y.toFixed(2)}`)
        ctx.fillStyle = '#064e3b'
        ctx.fillText(label, px + 5, cy)
      })
      ctx.restore()
    }
  }, [map, robotPose, pois])

  return (
    <section className="bg-white p-4">
      <div className="flex items-center justify-between mb-2 gap-3">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map</h2>
      </div>
      <div className="h-96 bg-gradient-to-br from-blue-50 to-indigo-50 rounded-lg overflow-hidden relative flex items-center justify-center">
        {!map && (
          <div className="text-center p-4 text-blue-700">
            <div className="font-semibold mb-1">No map available</div>
            <div className="text-sm">Start SLAM or Localization to publish <code>/map</code>.</div>
          </div>
        )}
        <canvas ref={canvasRef} className="w-full h-full object-contain" style={{ imageRendering: 'pixelated' }} />
      </div>
    </section>
  )
}

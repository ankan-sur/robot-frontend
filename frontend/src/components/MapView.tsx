import { useEffect, useRef } from 'react'
import { useMap, useRobotPose, usePointsOfInterest } from '../ros/hooks'

type Props = {
  embedded?: boolean // when true, render without outer section/header for tab embedding
}

export function MapView({ embedded = false }: Props) {
  const map = useMap()
  const robotPose = useRobotPose() // subscribe to /robot/pose (map frame)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const pois = usePointsOfInterest()

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

  const content = (
    <div className="relative">
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
          className="w-full h-full object-contain"
          style={{ imageRendering: 'pixelated' }}
        />
      </div>
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

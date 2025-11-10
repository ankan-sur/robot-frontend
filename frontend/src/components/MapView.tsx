import { useEffect, useRef } from 'react'
import { useMap, useOdom } from '../ros/hooks'

export function MapView() {
  const map = useMap()
  const odom = useOdom()
  const canvasRef = useRef<HTMLCanvasElement>(null)

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
  }, [map, odom])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map</h2>
      <div className="h-96 bg-gradient-to-br from-blue-50 to-indigo-50 rounded-lg border-2 border-blue-300 overflow-hidden relative">
        <canvas ref={canvasRef} className="w-full h-full object-contain" style={{ imageRendering: 'pixelated' }} />
      </div>
    </section>
  )
}

import { useRef, useEffect } from 'react'
import { useMap, useOdom } from '../ros/hooks'

type Props = {
  position?: { x: number; y: number; z: number }
}

export function MapView({ position }: Props) {
  const map = useMap()
  const odom = useOdom()
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const robotPos = position || odom?.pose.pose.position || { x: 0, y: 0, z: 0 }
  const robotOri = odom?.pose.pose.orientation || { x: 0, y: 0, z: 0, w: 1 }

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || !map) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const { width, height, resolution, origin } = map.info
    const data = map.data

    // Set canvas size
    canvas.width = width
    canvas.height = height

    // Create image data
    const imageData = ctx.createImageData(width, height)

    // Draw map
    for (let i = 0; i < data.length; i++) {
      const value = data[i]
      const idx = i * 4

      if (value === -1) {
        // Unknown - gray
        imageData.data[idx] = 128
        imageData.data[idx + 1] = 128
        imageData.data[idx + 2] = 128
        imageData.data[idx + 3] = 255
      } else if (value === 0) {
        // Free space - white
        imageData.data[idx] = 255
        imageData.data[idx + 1] = 255
        imageData.data[idx + 2] = 255
        imageData.data[idx + 3] = 255
      } else {
        // Occupied - black
        imageData.data[idx] = 0
        imageData.data[idx + 1] = 0
        imageData.data[idx + 2] = 0
        imageData.data[idx + 3] = 255
      }
    }

    ctx.putImageData(imageData, 0, 0)

    // Draw robot position
    const mapX = (robotPos.x - origin.position.x) / resolution
    const mapY = (robotPos.y - origin.position.y) / resolution
    const canvasY = height - mapY // Flip Y axis

    if (mapX >= 0 && mapX < width && canvasY >= 0 && canvasY < height) {
      ctx.save()
      ctx.translate(mapX, canvasY)
      
      // Calculate yaw from quaternion
      const yaw = 2 * Math.atan2(robotOri.z, robotOri.w)
      ctx.rotate(yaw)

      // Draw robot as a triangle
      ctx.fillStyle = '#a855f7' // Purple
      ctx.beginPath()
      ctx.moveTo(0, -8)
      ctx.lineTo(-5, 5)
      ctx.lineTo(5, 5)
      ctx.closePath()
      ctx.fill()

      // Draw direction line
      ctx.strokeStyle = '#ec4899' // Pink
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(0, 0)
      ctx.lineTo(0, -15)
      ctx.stroke()

      ctx.restore()
    }
  }, [map, robotPos, robotOri])

  if (!map) {
    return (
      <section className="rounded-lg border-2 border-amber-300 bg-gradient-to-br from-amber-50 to-orange-50 p-4 shadow-lg">
        <h2 className="text-lg font-semibold mb-3 bg-gradient-to-r from-amber-600 to-orange-600 bg-clip-text text-transparent">Map</h2>
        <div className="h-64 bg-gradient-to-br from-amber-100 to-orange-100 rounded-lg border-2 border-amber-300 flex items-center justify-center text-sm font-medium text-amber-800">
          Waiting for map...
        </div>
      </section>
    )
  }

  return (
    <section className="rounded-lg border-2 border-emerald-400 bg-gradient-to-br from-white to-emerald-50 p-4 shadow-lg">
      <h2 className="text-lg font-semibold mb-3 bg-gradient-to-r from-emerald-600 to-teal-600 bg-clip-text text-transparent">Navigation Map</h2>
      <div className="h-96 bg-gradient-to-br from-emerald-50 to-teal-50 rounded-lg border-2 border-emerald-300 overflow-hidden relative">
        <canvas
          ref={canvasRef}
          className="w-full h-full object-contain"
          style={{ imageRendering: 'pixelated' }}
        />
        <div className="absolute bottom-2 left-2 bg-gradient-to-r from-purple-600 to-pink-600 text-white text-xs px-3 py-1.5 rounded-lg shadow-lg font-semibold">
          Pos: ({robotPos.x.toFixed(2)}, {robotPos.y.toFixed(2)})
        </div>
      </div>
    </section>
  )
}

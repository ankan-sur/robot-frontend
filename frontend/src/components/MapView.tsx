import { useRef, useEffect } from 'react'
import { useMap, useOdom } from '../ros/hooks'

type Props = {
  position?: { x: number; y: number; z: number }
}

export function MapView({ position }: Props) {
  const map = useMap()
  const odom = useOdom()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const cachedMapRef = useRef<HTMLCanvasElement | null>(null)

  const robotPos = position || odom?.pose.pose.position || { x: 0, y: 0, z: 0 }
  const robotOri = odom?.pose.pose.orientation || { x: 0, y: 0, z: 0, w: 1 }
  const posX = robotPos.x
  const posY = robotPos.y
  const oriZ = robotOri.z
  const oriW = robotOri.w

  useEffect(() => {
    if (!map) {
      cachedMapRef.current = null
      return
    }

    const { width, height } = map.info
    const data = map.data
    const offscreen = document.createElement('canvas')
    offscreen.width = width
    offscreen.height = height
    const ctx = offscreen.getContext('2d')
    if (!ctx) return

    const imageData = ctx.createImageData(width, height)

    for (let i = 0; i < data.length; i++) {
      const value = data[i]
      const idx = i * 4

      if (value === -1) {
        imageData.data[idx] = 128
        imageData.data[idx + 1] = 128
        imageData.data[idx + 2] = 128
        imageData.data[idx + 3] = 255
      } else if (value === 0) {
        imageData.data[idx] = 255
        imageData.data[idx + 1] = 255
        imageData.data[idx + 2] = 255
        imageData.data[idx + 3] = 255
      } else {
        imageData.data[idx] = 0
        imageData.data[idx + 1] = 0
        imageData.data[idx + 2] = 0
        imageData.data[idx + 3] = 255
      }
    }

    ctx.putImageData(imageData, 0, 0)
    cachedMapRef.current = offscreen
  }, [map])

  useEffect(() => {
    const canvas = canvasRef.current
    const cached = cachedMapRef.current
    if (!canvas || !map || !cached) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const { width, height, resolution, origin } = map.info
    canvas.width = width
    canvas.height = height

    ctx.clearRect(0, 0, width, height)
    ctx.drawImage(cached, 0, 0)

    const mapX = (posX - origin.position.x) / resolution
    const mapY = (posY - origin.position.y) / resolution
    const canvasY = height - mapY

    if (mapX >= 0 && mapX < width && canvasY >= 0 && canvasY < height) {
      ctx.save()
      ctx.translate(mapX, canvasY)

      const yaw = 2 * Math.atan2(oriZ, oriW)
      ctx.rotate(yaw)

      ctx.fillStyle = '#2563eb'
      ctx.beginPath()
      ctx.moveTo(0, -8)
      ctx.lineTo(-5, 5)
      ctx.lineTo(5, 5)
      ctx.closePath()
      ctx.fill()

      ctx.strokeStyle = '#0ea5e9'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(0, 0)
      ctx.lineTo(0, -15)
      ctx.stroke()

      ctx.restore()
    }
  }, [map, posX, posY, oriZ, oriW])

  if (!map) {
    return (
      <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
        <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Map</h2>
        <div className="h-64 bg-gradient-to-br from-blue-100 to-indigo-100 rounded-lg border-2 border-blue-300 flex items-center justify-center text-base font-medium text-blue-800">
          Waiting for map...
        </div>
      </section>
    )
  }

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map</h2>
      <div className="h-96 bg-gradient-to-br from-blue-50 to-indigo-50 rounded-lg border-2 border-blue-300 overflow-hidden relative">
        <canvas
          ref={canvasRef}
          className="w-full h-full object-contain"
          style={{ imageRendering: 'pixelated' }}
        />
        <div className="absolute bottom-2 left-2 bg-gradient-to-r from-blue-600 to-indigo-600 text-white text-sm px-3 py-1.5 rounded-lg shadow-lg font-semibold">
          Pos: ({robotPos.x.toFixed(2)}, {robotPos.y.toFixed(2)})
        </div>
      </div>
    </section>
  )
}

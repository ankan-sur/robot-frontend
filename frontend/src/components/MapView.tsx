import { useEffect, useId, useMemo, useRef, useState } from 'react'
import type { Odometry } from '../ros/hooks'
import { loadRos2d } from '../lib/robotWebTools'
import { ros } from '../ros/ros'
import { ROS_CONFIG } from '../ros/config'

function quaternionToYaw(q: { x: number; y: number; z: number; w: number }) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

type Props = {
  odom?: Odometry | null
}

export function MapView({ odom }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const viewerRef = useRef<any>(null)
  const robotArrowRef = useRef<any>(null)
  const containerId = useId().replace(/[:]/g, '')
  const [status, setStatus] = useState<'loading' | 'ready' | 'error'>('loading')
  const [error, setError] = useState<string | null>(null)

  const position = odom?.pose?.pose?.position ?? { x: 0, y: 0, z: 0 }
  const orientation = odom?.pose?.pose?.orientation ?? { x: 0, y: 0, z: 0, w: 1 }
  const yaw = useMemo(() => quaternionToYaw(orientation), [orientation.x, orientation.y, orientation.z, orientation.w])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    container.innerHTML = ''
    setStatus('loading')
    setError(null)
    let cancelled = false

    const width = container.clientWidth || 720
    const height = container.clientHeight || 384

    const init = async () => {
      try {
        const ROS2D = await loadRos2d()
        if (cancelled || !containerRef.current) return

        const viewer = new ROS2D.Viewer({
          divID: containerId,
          width,
          height,
          background: '#020617'
        })

        const gridClient = new ROS2D.OccupancyGridClient({
          ros,
          rootObject: viewer.scene,
          topic: ROS_CONFIG.topics.map,
          continuous: true
        })

        gridClient.on('change', () => {
          viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height)
          viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y)
          setStatus('ready')
        })

        const robotArrow = new ROS2D.NavigationArrow({
          size: 0.6,
          strokeSize: 0.02,
          strokeColor: createjs.Graphics.getRGB(37, 99, 235),
          fillColor: createjs.Graphics.getRGB(99, 102, 241),
          pulse: true
        })

        viewer.scene.addChild(robotArrow)
        viewerRef.current = viewer
        robotArrowRef.current = robotArrow
      } catch (err: any) {
        if (!cancelled) {
          setStatus('error')
          setError(err?.message || 'Failed to load map viewer')
        }
      }
    }

    init()

    return () => {
      cancelled = true
      viewerRef.current = null
      robotArrowRef.current = null
      container.innerHTML = ''
    }
  }, [containerId])

  useEffect(() => {
    const arrow = robotArrowRef.current
    if (!arrow) return
    arrow.x = position.x
    arrow.y = position.y
    arrow.rotation = (-yaw * 180) / Math.PI
  }, [position.x, position.y, yaw])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map</h2>
      <div className="h-96 bg-slate-900/50 rounded-lg border-2 border-blue-300 overflow-hidden relative">
        <div id={containerId} ref={containerRef} className="w-full h-full" />
        {status === 'loading' && (
          <div className="absolute inset-0 flex items-center justify-center text-blue-100">
            Loading /map …
          </div>
        )}
        {status === 'error' && (
          <div className="absolute inset-0 flex items-center justify-center text-red-200 text-center px-4">
            {error}
          </div>
        )}
        <div className="absolute bottom-2 left-2 bg-gradient-to-r from-blue-600 to-indigo-600 text-white text-sm px-3 py-1.5 rounded-lg shadow">
          X: {position.x.toFixed(2)} | Y: {position.y.toFixed(2)}
        </div>
      </div>
    </section>
  )
}

import { useEffect, useId, useMemo, useRef, useState } from 'react'
import type { Odometry } from '../ros/hooks'
import { getRos2d } from '../lib/robotWebTools'
import { ros } from '../ros/ros'
import { ROS_CONFIG } from '../ros/config'
import * as createjs from '@createjs/easeljs'

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
  const [mapReady, setMapReady] = useState(false)

  const position = odom?.pose?.pose?.position ?? { x: 0, y: 0, z: 0 }
  const orientation = odom?.pose?.pose?.orientation ?? { x: 0, y: 0, z: 0, w: 1 }
  const posX = position.x
  const posY = position.y
  const oriX = orientation.x
  const oriY = orientation.y
  const oriZ = orientation.z
  const oriW = orientation.w
  const yaw = useMemo(() => quaternionToYaw({ x: oriX, y: oriY, z: oriZ, w: oriW }), [oriX, oriY, oriZ, oriW])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    container.innerHTML = ''
    setMapReady(false)
    let cancelled = false

    const width = container.clientWidth || 720
    const height = container.clientHeight || 384

    const init = async () => {
      try {
        const ros2d: any = await getRos2d()
        if (cancelled || !containerRef.current) return

        const viewer = new ros2d.Viewer({
          divID: containerId,
          width,
          height,
          background: '#020617'
        })

        const gridClient = new ros2d.OccupancyGridClient({
          ros,
          rootObject: viewer.scene,
          topic: ROS_CONFIG.topics.map,
          continuous: true
        })

        gridClient.on('change', () => {
          viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height)
          viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y)
          setMapReady(true)
        })

        const robotArrow = new ros2d.NavigationArrow({
          size: 0.6,
          strokeSize: 0.02,
          strokeColor: createjs.Graphics.getRGB(37, 99, 235),
          fillColor: createjs.Graphics.getRGB(99, 102, 241),
          pulse: true
        })

        viewer.scene.addChild(robotArrow)
        robotArrowRef.current = robotArrow
        viewerRef.current = viewer
      } catch (error) {
        console.error('Failed to initialize ros2d viewer', error)
      }
    }

    init()

    return () => {
      cancelled = true
      robotArrowRef.current = null
      viewerRef.current = null
      container.innerHTML = ''
    }
  }, [containerId])

  useEffect(() => {
    const arrow = robotArrowRef.current
    const viewer = viewerRef.current
    if (!arrow || !viewer) return

    arrow.x = posX
    arrow.y = posY

    if (viewer.scene?.rosQuaternionToGlobalTheta) {
      arrow.rotation = viewer.scene.rosQuaternionToGlobalTheta({ x: oriX, y: oriY, z: oriZ, w: oriW })
    } else {
      arrow.rotation = (-yaw * 180) / Math.PI
    }
  }, [posX, posY, yaw, oriX, oriY, oriZ, oriW])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Navigation Map (ros2d)</h2>
      <div className="h-96 bg-slate-900/50 rounded-lg border-2 border-blue-300 overflow-hidden relative">
        <div id={containerId} ref={containerRef} className="w-full h-full" />
        {!mapReady && (
          <div className="absolute inset-0 flex items-center justify-center text-base font-medium text-blue-100 bg-slate-900/70">
            Loading /map …
          </div>
        )}
        <div className="absolute bottom-2 left-2 bg-gradient-to-r from-blue-600 to-indigo-600 text-white text-sm px-3 py-1.5 rounded-lg shadow-lg font-semibold">
          X: {posX.toFixed(2)} | Y: {posY.toFixed(2)}
        </div>
      </div>
    </section>
  )
}

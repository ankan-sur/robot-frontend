import { useEffect, useId, useMemo, useRef } from 'react'
import * as THREE from 'three'
import type { Odometry } from '../ros/hooks'
import { getRos3d } from '../lib/robotWebTools'

const toYaw = (q: { x: number; y: number; z: number; w: number }) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

type Props = {
  odom?: Odometry | null
}

export function SpatialView({ odom }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const viewerRef = useRef<any>(null)
  const robotArrowRef = useRef<any>(null)
  const containerId = useId().replace(/[:]/g, '')

  const position = odom?.pose?.pose?.position ?? { x: 0, y: 0, z: 0 }
  const orientation = odom?.pose?.pose?.orientation ?? { x: 0, y: 0, z: 0, w: 1 }
  const posX = position.x
  const posY = position.y
  const posZ = position.z
  const oriX = orientation.x
  const oriY = orientation.y
  const oriZ = orientation.z
  const oriW = orientation.w
  const yaw = useMemo(() => toYaw({ x: oriX, y: oriY, z: oriZ, w: oriW }), [oriX, oriY, oriZ, oriW])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    container.innerHTML = ''
    const width = container.clientWidth || 480
    const height = container.clientHeight || 320
    let cancelled = false

    const init = async () => {
      try {
        const ROS3D: any = await getRos3d()
        if (cancelled || !containerRef.current) return

        const viewer = new ROS3D.Viewer({
          divID: containerId,
          width,
          height,
          antialias: true,
          background: '#020617',
          cameraPose: { x: 2.2, y: 2.2, z: 1.4 },
          near: 0.05,
          far: 100
        })

        viewer.addObject(
          new ROS3D.Grid({
            cellSize: 0.5,
            numCells: 20,
            color: '#1e293b'
          })
        )
        viewer.addObject(
          new ROS3D.Axes({
            shaftLength: 0.7,
            headLength: 0.12,
            shaftRadius: 0.015,
            headRadius: 0.03
          })
        )

        const materialFactory = ROS3D.makeColorMaterial || ((r: number, g: number, b: number, a: number) => new THREE.MeshPhongMaterial({ color: new THREE.Color(r, g, b).getHex(), opacity: a }))
        const robotArrow = new ROS3D.Arrow({
          length: 0.9,
          headLength: 0.25,
          shaftDiameter: 0.04,
          headDiameter: 0.12,
          material: materialFactory(0.25, 0.63, 1, 1)
        })
        viewer.addObject(robotArrow)

        viewerRef.current = viewer
        robotArrowRef.current = robotArrow
      } catch (error) {
        console.error('Failed to initialize ros3d viewer', error)
      }
    }

    init()

    return () => {
      cancelled = true
      viewerRef.current?.stop?.()
      viewerRef.current = null
      robotArrowRef.current = null
      container.innerHTML = ''
    }
  }, [containerId])

  useEffect(() => {
    const arrow = robotArrowRef.current
    if (!arrow) return

    arrow.position.set(posX, posY, posZ)
    const newDir = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0)
    arrow.setDirection(newDir.normalize())
  }, [posX, posY, posZ, yaw])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Spatial Pose (ros3d)</h2>
      <div className="h-80 bg-slate-900/50 rounded-lg border-2 border-indigo-300 overflow-hidden relative">
        <div id={containerId} ref={containerRef} className="w-full h-full" />
        <div className="absolute bottom-2 left-2 bg-slate-900/90 text-white text-xs px-3 py-1.5 rounded-lg shadow">
          <div>Heading: {(yaw * 180 / Math.PI).toFixed(1)}°</div>
          <div>Pose: ({posX.toFixed(2)}, {posY.toFixed(2)}, {posZ.toFixed(2)})</div>
        </div>
      </div>
    </section>
  )
}

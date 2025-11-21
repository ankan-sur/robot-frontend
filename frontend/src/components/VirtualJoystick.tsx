import { useEffect, useRef, useState } from 'react'
import { useCmdVel, useRosConnection } from '../ros/hooks'

type Props = {
  maxLinear: number // m/s
  maxAngular: number // rad/s
  rateHz?: number // publish rate
  deadzonePx?: number
  radiusPx?: number
}

export default function VirtualJoystick({ maxLinear, maxAngular, rateHz = 20, deadzonePx = 12, radiusPx = 80 }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const knobRef = useRef<HTMLDivElement>(null)
  const [active, setActive] = useState(false)
  const centerRef = useRef<{ x: number; y: number }>({ x: 0, y: 0 })
  const vecRef = useRef<{ dx: number; dy: number }>({ dx: 0, dy: 0 })
  const intervalRef = useRef<number | null>(null)
  const { send } = useCmdVel()
  const { connected } = useRosConnection()

  // Publish loop
  useEffect(() => {
    if (active && connected) {
      const period = Math.max(10, Math.round(1000 / rateHz))
      intervalRef.current = window.setInterval(() => {
        const { dx, dy } = vecRef.current
        const r = Math.hypot(dx, dy)
        if (r < deadzonePx) {
          send(0, 0, maxLinear, maxAngular)
          return
        }
        const nx = Math.max(-1, Math.min(1, dx / radiusPx)) // right +
        const ny = Math.max(-1, Math.min(1, dy / radiusPx)) // down +
        const vx = -ny * maxLinear // up = forward
        const wz = -nx * maxAngular // right = turn right (inverted: right is negative angular)
        send(vx, wz, maxLinear, maxAngular)
      }, period)
    }
    return () => {
      if (intervalRef.current != null) {
        window.clearInterval(intervalRef.current)
        intervalRef.current = null
      }
    }
  }, [active, connected, rateHz, maxLinear, maxAngular, deadzonePx, radiusPx, send])

  const updateKnob = (dx: number, dy: number) => {
    const r = Math.hypot(dx, dy)
    let kx = dx, ky = dy
    if (r > radiusPx) {
      const s = radiusPx / r
      kx *= s
      ky *= s
    }
    if (knobRef.current) {
      knobRef.current.style.transform = `translate(${kx}px, ${ky}px)`
    }
    vecRef.current = { dx: kx, dy: ky }
  }

  const resetKnob = () => {
    if (knobRef.current) knobRef.current.style.transform = 'translate(0px, 0px)'
    vecRef.current = { dx: 0, dy: 0 }
  }

  const onPointerDown = (e: React.PointerEvent) => {
    if (!containerRef.current) return
    const rect = containerRef.current.getBoundingClientRect()
    centerRef.current = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 }
    setActive(true)
    ;(e.target as HTMLElement).setPointerCapture(e.pointerId)
    onPointerMove(e)
  }

  const onPointerMove = (e: React.PointerEvent) => {
    if (!active) return
    const { x, y } = centerRef.current
    const dx = e.clientX - x
    const dy = e.clientY - y
    updateKnob(dx, dy)
  }

  const onPointerUp = (e: React.PointerEvent) => {
    setActive(false)
    resetKnob()
    // final zero
    send(0, 0, maxLinear, maxAngular)
  }

  return (
    <div className="flex flex-col items-center gap-2">
      <div
        ref={containerRef}
        className="relative w-56 h-56 rounded-full bg-slate-100 border border-slate-300 touch-none select-none"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
      >
        <div className="absolute inset-1 rounded-full border border-slate-200" />
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="w-1 h-1 bg-slate-300 rounded-full" />
        </div>
        <div
          ref={knobRef}
          className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 w-16 h-16 rounded-full bg-blue-100 border-2 border-blue-300 shadow-md"
          style={{ transition: active ? 'none' as any : 'transform 120ms ease-out' }}
        />
      </div>
      <div className="text-[11px] text-slate-500">Drag: up/down = linear, left/right = turn</div>
    </div>
  )
}

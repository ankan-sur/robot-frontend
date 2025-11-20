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
  const activeRef = useRef(false) // Use ref for synchronous access
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
        const wz = nx * maxAngular // right = turn right (negative is left)
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

  const handleMove = (clientX: number, clientY: number) => {
    if (!activeRef.current) return
    const { x, y } = centerRef.current
    const dx = clientX - x
    const dy = clientY - y
    updateKnob(dx, dy)
  }

  const handleStart = (clientX: number, clientY: number) => {
    if (!containerRef.current) return
    const rect = containerRef.current.getBoundingClientRect()
    centerRef.current = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 }
    activeRef.current = true
    setActive(true)
    handleMove(clientX, clientY)
  }

  const handleEnd = () => {
    activeRef.current = false
    setActive(false)
    resetKnob()
    send(0, 0, maxLinear, maxAngular)
  }

  // Touch event handlers
  const onTouchStart = (e: React.TouchEvent) => {
    e.preventDefault()
    if (e.touches.length > 0) {
      const touch = e.touches[0]
      handleStart(touch.clientX, touch.clientY)
    }
  }

  const onTouchMove = (e: React.TouchEvent) => {
    e.preventDefault()
    if (e.touches.length > 0) {
      const touch = e.touches[0]
      handleMove(touch.clientX, touch.clientY)
    }
  }

  const onTouchEnd = (e: React.TouchEvent) => {
    e.preventDefault()
    handleEnd()
  }

  // Mouse event handlers (for desktop)
  const onMouseDown = (e: React.MouseEvent) => {
    e.preventDefault()
    handleStart(e.clientX, e.clientY)
  }

  const onMouseMove = (e: React.MouseEvent) => {
    handleMove(e.clientX, e.clientY)
  }

  const onMouseUp = (e: React.MouseEvent) => {
    e.preventDefault()
    handleEnd()
  }

  const onMouseLeave = () => {
    if (activeRef.current) {
      handleEnd()
    }
  }

  return (
    <div className="flex flex-col items-center gap-2">
      <div
        ref={containerRef}
        className="relative w-56 h-56 rounded-full bg-slate-700 border-2 border-slate-600 touch-none select-none"
        onTouchStart={onTouchStart}
        onTouchMove={onTouchMove}
        onTouchEnd={onTouchEnd}
        onTouchCancel={onTouchEnd}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseLeave}
      >
        <div className="absolute inset-2 rounded-full border border-slate-600" />
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="w-2 h-2 bg-slate-500 rounded-full" />
        </div>
        <div
          ref={knobRef}
          className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 w-16 h-16 rounded-full bg-blue-600 border-2 border-blue-500 shadow-lg pointer-events-none"
          style={{ transition: active ? 'none' as any : 'transform 120ms ease-out' }}
        />
      </div>
      <div className="text-sm text-slate-400">Drag: up/down = linear, left/right = turn</div>
    </div>
  )
}

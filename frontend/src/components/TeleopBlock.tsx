import { useEffect, useRef, useState } from 'react'
import { topics } from '../ros/ros'
import { useRosConnection } from '../ros/hooks'
import { getMode } from '../ros/services'

export function TeleopBlock() {
  const { connected } = useRosConnection()
  const [currentMode, setCurrentMode] = useState<string | null>(null)
  // Sliders (adjust while holding for immediate effect)
  const [linSet, setLinSet] = useState(0.3)
  const [angSet, setAngSet] = useState(1.0)
  // Held command velocity
  const held = useRef({ vx: 0, wz: 0 })
  const intervalRef = useRef<number | null>(null)
  const keysRef = useRef<Set<string>>(new Set())
  const PUBLISH_INTERVAL_MS = 100 // 10Hz per clarification

  // Max speed caps (mode-dependent)
  const MAX_LIN_BASE = 2.5
  const MAX_ANG_BASE = 2.5
  const MAX_LIN_SLAM = 1.5
  const MAX_ANG_SLAM = 2.5
  const effectiveMaxLin = () => currentMode === 'slam' ? MAX_LIN_SLAM : MAX_LIN_BASE
  const effectiveMaxAng = () => currentMode === 'slam' ? MAX_ANG_SLAM : MAX_ANG_BASE

  // Fetch current mode periodically
  useEffect(() => {
    const fetchMode = async () => {
      try {
        const res = await getMode()
        setCurrentMode(res?.mode || null)
      } catch {
        setCurrentMode(null)
      }
    }
    fetchMode()
    const interval = setInterval(fetchMode, 5000) // refresh every 5s
    return () => clearInterval(interval)
  }, [])

  const publishTwist = () => {
    if (!connected) return
    const vx = Math.max(-effectiveMaxLin(), Math.min(effectiveMaxLin(), held.current.vx))
    const wz = Math.max(-effectiveMaxAng(), Math.min(effectiveMaxAng(), held.current.wz))
    topics.cmdVel.publish({
      linear: { x: vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: wz },
    } as any)
  }

  const startLoop = () => {
    if (!connected || intervalRef.current != null) return
    intervalRef.current = window.setInterval(publishTwist, PUBLISH_INTERVAL_MS)
  }

  const stopLoopIfIdle = () => {
    if (held.current.vx === 0 && held.current.wz === 0 && intervalRef.current != null) {
      window.clearInterval(intervalRef.current)
      intervalRef.current = null
    }
  }

  useEffect(() => {
    if (!connected && intervalRef.current != null) {
      window.clearInterval(intervalRef.current)
      intervalRef.current = null
      held.current = { vx: 0, wz: 0 }
    }
  }, [connected])

  const guard = (fn: () => void) => (e?: any) => {
    if (e?.preventDefault) e.preventDefault()
    if (!connected) return
    fn()
  }

  const lin = () => Math.max(0, Math.min(effectiveMaxLin(), linSet))
  const ang = () => Math.max(0, Math.min(effectiveMaxAng(), angSet))

  // If sliders change while moving, update held velocities immediately
  useEffect(() => {
    if (held.current.vx > 0) held.current.vx = lin()
    if (held.current.vx < 0) held.current.vx = -lin()
    if (held.current.wz > 0) held.current.wz = ang()
    if (held.current.wz < 0) held.current.wz = -ang()
  }, [linSet, angSet, currentMode])

  // Keyboard teleop (W/S forward/back, A/D rotate, arrows alias)
  useEffect(() => {
    const recompute = () => {
      // Linear
      if (keysRef.current.has('w') || keysRef.current.has('arrowup')) {
        held.current.vx = lin()
      } else if (keysRef.current.has('s') || keysRef.current.has('arrowdown')) {
        held.current.vx = -lin()
      } else {
        held.current.vx = 0
      }
      // Angular
      if (keysRef.current.has('a') || keysRef.current.has('arrowleft')) {
        held.current.wz = ang()
      } else if (keysRef.current.has('d') || keysRef.current.has('arrowright')) {
        held.current.wz = -ang()
      } else {
        held.current.wz = 0
      }
      if (held.current.vx !== 0 || held.current.wz !== 0) {
        startLoop()
      } else {
        publishTwist()
        stopLoopIfIdle()
      }
    }
    const down = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      if (["w","a","s","d","arrowup","arrowdown","arrowleft","arrowright"].includes(k)) {
        e.preventDefault()
        keysRef.current.add(k)
        recompute()
      }
    }
    const up = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      if (keysRef.current.delete(k)) {
        recompute()
      }
    }
    window.addEventListener('keydown', down, { passive: false })
    window.addEventListener('keyup', up)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
      keysRef.current.clear()
    }
  }, [connected, linSet, angSet, currentMode])

  return (
  <div className="space-y-3">
      <div className="flex flex-col gap-3 sm:flex-row sm:items-center sm:justify-between">
        <div className="flex items-center gap-2">
          <div className="text-base font-medium text-blue-800">Teleop</div>
          {currentMode && (
            <span className={`text-xs px-2 py-0.5 rounded-full font-semibold ${
              currentMode === 'slam' ? 'bg-yellow-200 text-yellow-800' :
              currentMode === 'localization' ? 'bg-blue-200 text-blue-800' :
              'bg-gray-200 text-gray-700'
            }`}>
              {currentMode.toUpperCase()}
            </span>
          )}
        </div>
        <div className="flex flex-col gap-2 w-full max-w-md sm:flex-row sm:gap-3 sm:w-auto">
          <div className="flex items-center gap-2 w-full">
            <label className="text-xs text-slate-600 whitespace-nowrap">Linear (m/s)</label>
            <input type="range" min={0} max={effectiveMaxLin()} step={0.05} value={linSet} onChange={(e)=>setLinSet(parseFloat(e.target.value))} className="w-full sm:w-28" />
            <span className="text-xs text-slate-600 w-10 text-right">{lin().toFixed(2)}</span>
          </div>
          <div className="flex items-center gap-2 w-full">
            <label className="text-xs text-slate-600 whitespace-nowrap">Angular (rad/s)</label>
            <input type="range" min={0} max={effectiveMaxAng()} step={0.05} value={angSet} onChange={(e)=>setAngSet(parseFloat(e.target.value))} className="w-full sm:w-28" />
            <span className="text-xs text-slate-600 w-10 text-right">{ang().toFixed(2)}</span>
          </div>
        </div>
      </div>

      {/* Virtual Joystick placeholder for mobile */}
      <div className="block sm:hidden">
        {/* TODO: Integrate <VirtualJoystick /> here for mobile touch control */}
        <div className="my-2 text-center text-xs text-slate-400">Touch joystick coming soon</div>
      </div>

      {/* Diamond layout for desktop */}
      <div className="hidden sm:grid grid-cols-3 gap-2 place-items-center select-none">
        <div />
        <button
          className="w-20 h-12 rounded bg-sky-100 hover:bg-sky-200 border border-sky-300 disabled:opacity-50"
          disabled={!connected}
          onMouseDown={guard(() => { held.current.vx = lin(); startLoop() })}
          onMouseUp={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
          onTouchStart={guard(() => { held.current.vx = lin(); startLoop() })}
          onTouchEnd={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
          title="Forward"
        >
          ▲
        </button>
        <div />

        <button
          className="w-20 h-12 rounded bg-sky-100 hover:bg-sky-200 border border-sky-300 disabled:opacity-50"
          disabled={!connected}
          onMouseDown={guard(() => { held.current.wz = ang(); startLoop() })}
          onMouseUp={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
          onTouchStart={guard(() => { held.current.wz = ang(); startLoop() })}
          onTouchEnd={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
          title="Turn left"
        >
          ◀
        </button>
        <button
          className="w-20 h-12 rounded bg-rose-100 hover:bg-rose-200 border border-rose-300 disabled:opacity-50"
          disabled={!connected}
          onClick={guard(() => { held.current = { vx: 0, wz: 0 }; publishTwist(); stopLoopIfIdle() })}
          title="Stop"
        >
          ■
        </button>
        <button
          className="w-20 h-12 rounded bg-sky-100 hover:bg-sky-200 border border-sky-300 disabled:opacity-50"
          disabled={!connected}
          onMouseDown={guard(() => { held.current.wz = -ang(); startLoop() })}
          onMouseUp={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
          onTouchStart={guard(() => { held.current.wz = -ang(); startLoop() })}
          onTouchEnd={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
          title="Turn right"
        >
          ▶
        </button>

        <div />
        <button
          className="w-20 h-12 rounded bg-sky-100 hover:bg-sky-200 border border-sky-300 disabled:opacity-50"
          disabled={!connected}
          onMouseDown={guard(() => { held.current.vx = -lin(); startLoop() })}
          onMouseUp={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
          onTouchStart={guard(() => { held.current.vx = -lin(); startLoop() })}
          onTouchEnd={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
          title="Reverse"
        >
          ▼
        </button>
        <div />
      </div>

      <div className="text-[11px] text-slate-500">Max lin {effectiveMaxLin().toFixed(1)} m/s (mode cap), ang {effectiveMaxAng().toFixed(1)} rad/s.</div>
    </div>
  )
}

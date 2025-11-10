import { useEffect, useRef, useState } from 'react'
import { topics } from '../ros/ros'
import { useRosConnection } from '../ros/hooks'

export function TeleopBlock() {
  const { connected } = useRosConnection()

  // Max speeds; UI slider scales these
  const MAX_LIN = 0.7
  const MAX_ANG = 1.4
  const [scale, setScale] = useState(0.5) // 50% by default

  const held = useRef({ vx: 0, wz: 0 })
  const intervalRef = useRef<number | null>(null)

  const publishTwist = () => {
    if (!connected) return
    topics.cmdVel.publish({
      linear: { x: held.current.vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: held.current.wz },
    } as any)
  }

  const startLoop = () => {
    if (!connected || intervalRef.current != null) return
    intervalRef.current = window.setInterval(publishTwist, 50)
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

  const lin = () => MAX_LIN * scale
  const ang = () => MAX_ANG * scale

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between">
        <div className="text-base font-medium text-blue-800">Teleop</div>
        <div className="flex items-center gap-2">
          <label className="text-xs text-slate-600">Speed</label>
          <input
            type="range"
            min={0.1}
            max={1}
            step={0.05}
            value={scale}
            onChange={(e) => setScale(parseFloat(e.target.value))}
            className="w-32"
          />
          <span className="text-xs text-slate-600 w-10 text-right">{Math.round(scale * 100)}%</span>
        </div>
      </div>

      {/* Diamond layout */}
      <div className="grid grid-cols-3 gap-2 place-items-center select-none">
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

      <div className="text-[11px] text-slate-500">
        Max lin {MAX_LIN.toFixed(1)} m/s, ang {MAX_ANG.toFixed(1)} rad/s; scaled by slider.
      </div>
    </div>
  )
}

import { useEffect, useRef, useState } from 'react'
import { topics } from '../ros/ros'
import { useRosConnection } from '../ros/hooks'

export function TeleopBlock() {
  const { connected } = useRosConnection()

  // Max speeds (typical safe defaults; adjust per robot)
  const MAX_LIN = 3.5
  const MAX_ANG = 3.5
  const [linSet, setLinSet] = useState(0.3)
  const [angSet, setAngSet] = useState(1.0)

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

  const lin = () => Math.max(0, Math.min(MAX_LIN, linSet))
  const ang = () => Math.max(0, Math.min(MAX_ANG, angSet))

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between">
        <div className="text-base font-medium text-blue-800">Teleop</div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <label className="text-xs text-slate-600">Linear (m/s)</label>
            <input type="range" min={0} max={MAX_LIN} step={0.05} value={linSet} onChange={(e)=>setLinSet(parseFloat(e.target.value))} className="w-28" />
            <span className="text-xs text-slate-600 w-10 text-right">{lin().toFixed(2)}</span>
          </div>
          <div className="flex items-center gap-2">
            <label className="text-xs text-slate-600">Angular (rad/s)</label>
            <input type="range" min={0} max={MAX_ANG} step={0.05} value={angSet} onChange={(e)=>setAngSet(parseFloat(e.target.value))} className="w-28" />
            <span className="text-xs text-slate-600 w-10 text-right">{ang().toFixed(2)}</span>
          </div>
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

      <div className="text-[11px] text-slate-500">Max lin {MAX_LIN.toFixed(1)} m/s, ang {MAX_ANG.toFixed(1)} rad/s.</div>
    </div>
  )
}

import React, { useEffect, useRef, useState, useMemo, useCallback } from 'react'
import { topics } from '../ros/ros'
import { useRosConnection } from '../ros/hooks'
import { getMode } from '../ros/services'
import VirtualJoystick from './VirtualJoystick'

export function TeleopBlock() {
  const { connected } = useRosConnection()
  const [currentMode, setCurrentMode] = useState<string | null>(null)
  const [linSet, setLinSet] = useState(0.3)
  const [angSet, setAngSet] = useState(1.0)
  const [showSettings, setShowSettings] = useState(false)
  const held = useRef({ vx: 0, wz: 0 })
  const intervalRef = useRef<number | null>(null)
  const keysRef = useRef<Set<string>>(new Set())
  const PUBLISH_INTERVAL_MS = 100

  const MAX_LIN_BASE = 2.5
  const MAX_ANG_BASE = 2.5
  const MAX_LIN_SLAM = 1.5
  const MAX_ANG_SLAM = 2.5
  const effectiveMaxLin = useMemo(() => currentMode === 'slam' ? MAX_LIN_SLAM : MAX_LIN_BASE, [currentMode])
  const effectiveMaxAng = useMemo(() => currentMode === 'slam' ? MAX_ANG_SLAM : MAX_ANG_BASE, [currentMode])

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
    const interval = setInterval(fetchMode, 5000)
    return () => clearInterval(interval)
  }, [])

  const publishTwist = useCallback(() => {
    if (!connected) return
    const vx = Math.max(-effectiveMaxLin, Math.min(effectiveMaxLin, held.current.vx))
    const wz = Math.max(-effectiveMaxAng, Math.min(effectiveMaxAng, held.current.wz))
    topics.cmdVel.publish({
      linear: { x: vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: wz },
    } as any)
  }, [connected, effectiveMaxLin, effectiveMaxAng])

  const startLoop = useCallback(() => {
    if (!connected || intervalRef.current != null) return
    intervalRef.current = window.setInterval(publishTwist, PUBLISH_INTERVAL_MS)
  }, [connected, publishTwist])

  const stopLoopIfIdle = useCallback(() => {
    if (held.current.vx === 0 && held.current.wz === 0 && intervalRef.current != null) {
      window.clearInterval(intervalRef.current)
      intervalRef.current = null
    }
  }, [])

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

  const lin = useMemo(() => Math.max(0, Math.min(effectiveMaxLin, linSet)), [effectiveMaxLin, linSet])
  const ang = useMemo(() => Math.max(0, Math.min(effectiveMaxAng, angSet)), [effectiveMaxAng, angSet])

  useEffect(() => {
    if (held.current.vx > 0) held.current.vx = lin
    if (held.current.vx < 0) held.current.vx = -lin
    if (held.current.wz > 0) held.current.wz = ang
    if (held.current.wz < 0) held.current.wz = -ang
  }, [lin, ang])

  useEffect(() => {
    const recompute = () => {
      if (keysRef.current.has('w') || keysRef.current.has('arrowup')) {
        held.current.vx = lin
      } else if (keysRef.current.has('s') || keysRef.current.has('arrowdown')) {
        held.current.vx = -lin
      } else {
        held.current.vx = 0
      }
      if (keysRef.current.has('a') || keysRef.current.has('arrowleft')) {
        held.current.wz = ang
      } else if (keysRef.current.has('d') || keysRef.current.has('arrowright')) {
        held.current.wz = -ang
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
  }, [lin, ang, publishTwist, startLoop, stopLoopIfIdle])

  return (
    <div className="space-y-3">
      {/* Header with Settings */}
      <div className="flex items-center justify-between">
        <div className="text-base font-semibold text-slate-300">Teleop</div>
        <button
          onClick={() => setShowSettings(!showSettings)}
          className="px-2 py-1 text-sm bg-slate-700 hover:bg-slate-600 text-slate-300 rounded"
        >
          {showSettings ? 'Hide' : 'Speed ⚙️'}
        </button>
      </div>

      {/* Settings Panel (collapsible) */}
      {showSettings && (
        <div className="bg-slate-900 rounded p-3 space-y-2 border border-slate-700">
          <div>
            <div className="flex items-center justify-between mb-1">
              <label className="text-sm text-slate-400">Linear Speed</label>
              <span className="text-sm font-mono text-white">{lin.toFixed(2)} m/s</span>
            </div>
            <input 
              type="range" 
              min={0} 
              max={effectiveMaxLin} 
              step={0.05} 
              value={linSet} 
              onChange={(e) => setLinSet(parseFloat(e.target.value))} 
              className="w-full h-2 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
          <div>
            <div className="flex items-center justify-between mb-1">
              <label className="text-sm text-slate-400">Angular Speed</label>
              <span className="text-sm font-mono text-white">{ang.toFixed(2)} rad/s</span>
            </div>
            <input 
              type="range" 
              min={0} 
              max={effectiveMaxAng} 
              step={0.05} 
              value={angSet} 
              onChange={(e) => setAngSet(parseFloat(e.target.value))} 
              className="w-full h-2 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
          <div className="text-sm text-slate-500 pt-2 border-t border-slate-700">
            Max: {effectiveMaxLin.toFixed(1)} m/s linear, {effectiveMaxAng.toFixed(1)} rad/s angular
            {currentMode === 'slam' && ' (SLAM limits)'}
          </div>
        </div>
      )}

      {/* Virtual Joystick for Mobile, Button Pad for Desktop */}
      <div className="flex justify-center">
        {/* Joystick - show on mobile only */}
        <div className="block lg:hidden">
          <VirtualJoystick 
            maxLinear={lin}
            maxAngular={ang}
            rateHz={10}
          />
        </div>

        {/* Button Pad - show on desktop only */}
        <div className="hidden lg:block w-full">
          <div className="grid grid-cols-3 gap-2 select-none max-w-sm mx-auto">
            <div />
            <button
              className="h-14 rounded-lg bg-blue-600 hover:bg-blue-500 active:bg-blue-700 disabled:bg-slate-700 disabled:text-slate-600 text-white text-lg font-bold shadow-lg disabled:cursor-not-allowed transition-colors"
              disabled={!connected}
              onMouseDown={guard(() => { held.current.vx = lin; startLoop() })}
              onMouseUp={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
              title="Forward"
            >
              ▲
            </button>
            <div />

            <button
              className="h-14 rounded-lg bg-blue-600 hover:bg-blue-500 active:bg-blue-700 disabled:bg-slate-700 disabled:text-slate-600 text-white text-lg font-bold shadow-lg disabled:cursor-not-allowed transition-colors"
              disabled={!connected}
              onMouseDown={guard(() => { held.current.wz = ang; startLoop() })}
              onMouseUp={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
              title="Turn left"
            >
              ◀
            </button>
            <button
              className="h-14 rounded-lg bg-red-600 hover:bg-red-500 active:bg-red-700 disabled:bg-slate-700 disabled:text-slate-600 text-white text-lg font-bold shadow-lg disabled:cursor-not-allowed transition-colors"
              disabled={!connected}
              onClick={guard(() => { held.current = { vx: 0, wz: 0 }; publishTwist(); stopLoopIfIdle() })}
              title="Stop"
            >
              ■
            </button>
            <button
              className="h-14 rounded-lg bg-blue-600 hover:bg-blue-500 active:bg-blue-700 disabled:bg-slate-700 disabled:text-slate-600 text-white text-lg font-bold shadow-lg disabled:cursor-not-allowed transition-colors"
              disabled={!connected}
              onMouseDown={guard(() => { held.current.wz = -ang; startLoop() })}
              onMouseUp={guard(() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle() })}
              title="Turn right"
            >
              ▶
            </button>

            <div />
            <button
              className="h-14 rounded-lg bg-blue-600 hover:bg-blue-500 active:bg-blue-700 disabled:bg-slate-700 disabled:text-slate-600 text-white text-lg font-bold shadow-lg disabled:cursor-not-allowed transition-colors"
              disabled={!connected}
              onMouseDown={guard(() => { held.current.vx = -lin; startLoop() })}
              onMouseUp={guard(() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle() })}
              title="Reverse"
            >
              ▼
            </button>
            <div />
          </div>

          <div className="text-sm text-slate-500 text-center mt-2">
            Use WASD/Arrow keys • {lin.toFixed(2)} m/s, {ang.toFixed(2)} rad/s
          </div>
        </div>
      </div>
    </div>
  )
}

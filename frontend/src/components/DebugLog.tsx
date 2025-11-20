import { useEffect, useRef, useState } from 'react'
import { topics } from '../ros/ros'

export function DebugLog() {
  const [logs, setLogs] = useState<string[]>([])
  const [isConnected, setIsConnected] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const logEndRef = useRef<HTMLDivElement>(null)
  const [autoScroll, setAutoScroll] = useState(false)

  // Hold current connections
  const rosTopicRef = useRef<{ topic?: any; cb?: (m:any)=>void } | null>(null)

  useEffect(() => {
    // Cleanup previous
    // Unsubscribe from previous
    try {
      if (rosTopicRef.current?.topic && rosTopicRef.current?.cb) {
        rosTopicRef.current.topic.unsubscribe(rosTopicRef.current.cb)
      }
    } catch {}
    rosTopicRef.current = null

    setLogs([])
    setError(null)
    setIsConnected(false)

    // ROS topic subscription
    const add = (s: string) => setLogs(prev => [...prev, s].slice(-1000))
    const onMsg = (m: any) => {
      // Preserve raw message as much as possible so bracketed levels like [ERROR] remain visible.
      const name = m?.name ?? m?.logger ?? 'rosout'
      const lvl = m?.level ?? m?.severity ?? ''
      const text = m?.msg ?? m?.message ?? m?.data ?? JSON.stringify(m)
      let line = `[${name}] ${text}`
      if (lvl !== '') line += ` (lvl ${lvl})`
      add(line)
    }
    try {
      topics.rosout.subscribe(onMsg); rosTopicRef.current = { topic: topics.rosout, cb: onMsg }
      setIsConnected(true)
    } catch (e: any) {
      setError(e?.message || 'Failed to subscribe')
      setIsConnected(false)
    }

    return () => {
      try {
        if (rosTopicRef.current?.topic && rosTopicRef.current?.cb) {
          rosTopicRef.current.topic.unsubscribe(rosTopicRef.current.cb)
        }
      } catch {}
      rosTopicRef.current = null
    }
  }, [])

  useEffect(() => {
    if (!autoScroll) return
    const container = logEndRef.current?.parentElement
    if (container) {
      logEndRef.current?.scrollIntoView({ behavior: 'auto', block: 'end' })
    }
  }, [logs, autoScroll])

  const clearLogs = () => setLogs([])

  return (
    <section className="rounded-lg bg-slate-800 border border-slate-700 p-4">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-base font-semibold text-slate-300">ROS Debug Log</h2>
        <div className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></span>
          <span className="text-xs text-slate-400">{isConnected ? 'Streaming' : 'Disconnected'}</span>
          <button onClick={clearLogs} className="px-2 py-1 text-xs rounded bg-slate-700 hover:bg-slate-600 text-slate-300 transition-colors">Clear</button>
          <label className="flex items-center gap-1 text-xs text-slate-400">
            <input type="checkbox" checked={autoScroll} onChange={(e)=>setAutoScroll(e.target.checked)} className="accent-blue-500" /> Auto-scroll
          </label>
        </div>
      </div>

      {error && (<div className="mb-2 p-2 text-sm text-red-300 bg-red-900/50 rounded border border-red-700">{error}</div>)}

      <div className="h-64 bg-slate-950 rounded-lg border border-slate-700 overflow-y-auto p-3 font-mono text-sm">
        {logs.length === 0 && !error && (<div className="text-slate-500 text-center py-8">Waiting for logs...</div>)}
        {logs.map((log, index) => {
          let textColor = 'text-slate-300'
          const lower = String(log).toLowerCase()
          if (lower.includes('error') || lower.includes('fatal')) textColor = 'text-red-400'
          else if (lower.includes('warn')) textColor = 'text-yellow-400'
          else if (lower.includes('info')) textColor = 'text-blue-400'
          return (<div key={index} className={`${textColor} mb-1 leading-relaxed`}>{log}</div>)
        })}
        <div ref={logEndRef} />
      </div>
    </section>
  )
}

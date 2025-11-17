import { useEffect, useRef, useState } from 'react'
import { topics } from '../ros/ros'

type TabKey = 'all' | 'errors'

export function DebugLog() {
  const [tab, setTab] = useState<TabKey>('all')
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
      let line = ''
      const name = m?.name ?? m?.logger ?? 'rosout'
      const lvl = m?.level ?? m?.severity ?? ''
      const text = m?.msg ?? m?.message ?? m?.data ?? JSON.stringify(m)
      line = `[${name}] ${text} ${lvl !== '' ? `(lvl ${lvl})` : ''}`
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
  const isErrLine = (s: string) => {
    const lower = s.toLowerCase()
    return lower.includes('error') || lower.includes('fatal') || lower.includes('warn')
  }
  const visible = tab === 'all' ? logs : logs.filter(isErrLine)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-lg font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Debug Log</h2>
        <div className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></span>
          <span className="text-xs text-blue-700">{isConnected ? 'Streaming' : 'Disconnected'}</span>
          <button onClick={clearLogs} className="px-2 py-1 text-xs rounded bg-blue-100 hover:bg-blue-200 text-blue-700 transition-colors">Clear</button>
          <label className="flex items-center gap-1 text-xs text-blue-700">
            <input type="checkbox" checked={autoScroll} onChange={(e)=>setAutoScroll(e.target.checked)} /> Auto-scroll
          </label>
        </div>
      </div>

      <div className="flex gap-2 mb-3">
        <TabButton active={tab==='all'} onClick={()=>setTab('all')}>All</TabButton>
        <TabButton active={tab==='errors'} onClick={()=>setTab('errors')}>Errors Only</TabButton>
      </div>

      {error && (<div className="mb-2 p-2 text-sm text-red-700 bg-red-50 rounded border border-red-200">{error}</div>)}

      <div className="h-64 bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 overflow-y-auto p-3 font-mono text-sm">
        {visible.length === 0 && !error && (<div className="text-blue-300 text-center py-8">Waiting for logs...</div>)}
        {visible.map((log, index) => {
          let textColor = 'text-blue-200'
          const lower = String(log).toLowerCase()
          if (lower.includes('error') || lower.includes('fatal')) textColor = 'text-red-300'
          else if (lower.includes('warn')) textColor = 'text-yellow-300'
          else if (lower.includes('info')) textColor = 'text-blue-300'
          return (<div key={index} className={`${textColor} mb-1 leading-relaxed`}>{log}</div>)
        })}
        <div ref={logEndRef} />
      </div>
    </section>
  )
}

function TabButton({ active, onClick, children }: { active: boolean; onClick?: ()=>void; children: any }) {
  return (
    <button onClick={onClick} className={`px-3 py-1.5 text-xs rounded border ${active ? 'bg-blue-600 text-white border-blue-600' : 'bg-white text-blue-700 border-blue-300'}`}>
      {children}
    </button>
  )
}

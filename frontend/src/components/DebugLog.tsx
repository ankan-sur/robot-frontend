import { useEffect, useState, useRef } from 'react'

const BACKEND_URL = import.meta.env.VITE_BACKEND_URL || 'http://localhost:8000'

export function DebugLog() {
  const [logs, setLogs] = useState<string[]>([])
  const [isConnected, setIsConnected] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const logEndRef = useRef<HTMLDivElement>(null)
  const eventSourceRef = useRef<EventSource | null>(null)

  useEffect(() => {
    // Connect to log stream
    const eventSource = new EventSource(`${BACKEND_URL}/logs/stream`)
    eventSourceRef.current = eventSource

    eventSource.onopen = () => {
      setIsConnected(true)
      setError(null)
    }

    eventSource.onmessage = (event) => {
      const line = event.data
      if (line && line.trim() !== '') {
        setLogs(prev => {
          const newLogs = [...prev, line]
          // Keep last 1000 lines to prevent memory issues
          return newLogs.slice(-1000)
        })
      }
    }

    eventSource.onerror = (err) => {
      console.error('Log stream error:', err)
      setIsConnected(false)
      setError('Failed to connect to log stream. Make sure backend is running and log.txt exists.')
    }

    return () => {
      eventSource.close()
      eventSourceRef.current = null
    }
  }, [])

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [logs])

  const clearLogs = () => {
    setLogs([])
  }

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-lg font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
          Debug Log
        </h2>
        <div className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></span>
          <span className="text-xs text-blue-700">
            {isConnected ? 'Streaming' : 'Disconnected'}
          </span>
          <button
            onClick={clearLogs}
            className="px-2 py-1 text-xs rounded bg-blue-100 hover:bg-blue-200 text-blue-700 transition-colors"
          >
            Clear
          </button>
        </div>
      </div>
      {error && (
        <div className="mb-2 p-2 text-sm text-red-700 bg-red-50 rounded border border-red-200">
          {error}
        </div>
      )}
      <div className="h-64 bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 overflow-y-auto p-3 font-mono text-sm">
        {logs.length === 0 && !error && (
          <div className="text-blue-300 text-center py-8">
            Waiting for logs...
          </div>
        )}
        {logs.map((log, index) => {
          // Color code different log levels
          let textColor = 'text-blue-200'
          if (log.toLowerCase().includes('error') || log.toLowerCase().includes('fatal')) {
            textColor = 'text-red-300'
          } else if (log.toLowerCase().includes('warn') || log.toLowerCase().includes('warning')) {
            textColor = 'text-yellow-300'
          } else if (log.toLowerCase().includes('info')) {
            textColor = 'text-blue-300'
          }
          
          return (
            <div key={index} className={`${textColor} mb-1 leading-relaxed`}>
              {log}
            </div>
          )
        })}
        <div ref={logEndRef} />
      </div>
    </section>
  )
}


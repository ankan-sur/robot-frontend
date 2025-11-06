import { useEffect, useRef, useState } from 'react'

type Telemetry = any

export function useTelemetry(wsUrl: string, statusUrl: string) {
  const [telemetry, setTelemetry] = useState<Telemetry | undefined>()
  const [ok, setOk] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)

  useEffect(() => {
    let cancelled = false
    let reconnectTimer: any

    async function pullOnce() {
      try {
        const res = await fetch(statusUrl)
        const json = await res.json()
        if (!cancelled) {
          setOk(Boolean(json?.ok))
          setTelemetry(json?.telemetry)
        }
      } catch {}
    }

    function connect() {
      try {
        const ws = new WebSocket(wsUrl)
        wsRef.current = ws
        ws.onopen = () => setOk(true)
        ws.onmessage = ev => {
          try {
            const data = JSON.parse(ev.data)
            setOk(Boolean(data?.ok))
            setTelemetry(data?.telemetry)
          } catch {}
        }
        ws.onclose = () => {
          setOk(false)
          reconnectTimer = setTimeout(connect, 1500)
        }
        ws.onerror = () => {
          try { ws.close() } catch {}
        }
      } catch {
        reconnectTimer = setTimeout(connect, 1500)
      }
    }

    // Kick off immediate REST pull, then connect WS
    pullOnce()
    connect()

    return () => {
      cancelled = true
      if (reconnectTimer) clearTimeout(reconnectTimer)
      wsRef.current?.close()
    }
  }, [wsUrl, statusUrl])

  return { telemetry, ok }
}





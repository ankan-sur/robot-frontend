import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

const SIGNALING_URL = import.meta.env.VITE_WEBRTC_SIGNALING_URL || ''
const STREAM_ID = import.meta.env.VITE_WEBRTC_STREAM_ID || ''

function parseIceServers() {
  const raw = import.meta.env.VITE_WEBRTC_ICE_SERVERS
  if (!raw) return undefined
  try {
    return JSON.parse(raw)
  } catch {
    console.warn('Failed to parse VITE_WEBRTC_ICE_SERVERS, expected JSON array')
    return undefined
  }
}

const iceServers = parseIceServers()

export default function VideoFeed() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const pcRef = useRef<RTCPeerConnection | null>(null)
  const [status, setStatus] = useState<'idle' | 'connecting' | 'connected' | 'error'>('idle')
  const [error, setError] = useState<string | null>(null)

  const canConnect = useMemo(() => Boolean(SIGNALING_URL), [])

  const cleanup = useCallback(() => {
    pcRef.current?.close()
    pcRef.current = null
    if (videoRef.current) {
      videoRef.current.srcObject = null
    }
  }, [])

  const connect = useCallback(async () => {
    if (!canConnect) return
    cleanup()
    setError(null)
    setStatus('connecting')

    const pc = new RTCPeerConnection({ iceServers })
    pcRef.current = pc

    pc.ontrack = (event) => {
      if (videoRef.current) {
        videoRef.current.srcObject = event.streams[0]
      }
    }

    pc.onconnectionstatechange = () => {
      if (!pcRef.current) return
      const state = pcRef.current.connectionState
      if (state === 'connected') setStatus('connected')
      if (state === 'failed' || state === 'disconnected') {
        setStatus('error')
        setError(`Peer connection ${state}`)
      }
    }

    try {
      const offer = await pc.createOffer({ offerToReceiveAudio: true, offerToReceiveVideo: true })
      await pc.setLocalDescription(offer)
      await waitForIceGathering(pc)

      const response = await fetch(SIGNALING_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sdp: pc.localDescription?.sdp,
          type: pc.localDescription?.type,
          streamId: STREAM_ID || undefined,
        })
      })

      if (!response.ok) {
        throw new Error(`Signaling server error (${response.status})`)
      }

      const answer = await response.json()
      if (!answer?.sdp) {
        throw new Error('Signaling server returned an invalid answer')
      }

      await pc.setRemoteDescription(answer)
      setStatus('connected')
    } catch (err: any) {
      console.error('WebRTC connection failed', err)
      setStatus('error')
      setError(err?.message || 'WebRTC connection failed')
      cleanup()
    }
  }, [canConnect, cleanup])

  useEffect(() => () => cleanup(), [cleanup])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">WebRTC Camera Feed</h2>
        <span className={`text-sm font-medium ${status === 'connected' ? 'text-green-700' : status === 'error' ? 'text-red-600' : 'text-blue-700'}`}>
          {status === 'connected' ? 'Streaming' : status === 'connecting' ? 'Connecting…' : status === 'error' ? 'Error' : 'Idle'}
        </span>
      </div>
      {!canConnect && (
        <div className="text-sm text-blue-700 bg-blue-50 border border-blue-200 rounded-lg p-3 mb-3">
          Set <code>VITE_WEBRTC_SIGNALING_URL</code> (and optional <code>VITE_WEBRTC_STREAM_ID</code>) to enable the WebRTC feed. Without it, the camera cannot connect.
        </div>
      )}
      {error && (
        <div className="text-sm text-red-700 bg-red-50 border border-red-200 rounded-lg p-2 mb-3">{error}</div>
      )}
      <div className="h-80 bg-slate-900 rounded-lg border-2 border-blue-500 flex items-center justify-center overflow-hidden relative">
        <video ref={videoRef} autoPlay playsInline controls={false} className="w-full h-full object-contain" />
        {!canConnect && (
          <div className="absolute inset-0 flex items-center justify-center text-blue-200 text-center px-4">
            Configure signaling to view the WebRTC stream.
          </div>
        )}
      </div>
      <div className="mt-3 flex gap-2">
        <button
          className="px-4 py-2 rounded bg-gradient-to-r from-blue-600 to-indigo-600 text-white font-medium disabled:opacity-50"
          onClick={connect}
          disabled={!canConnect || status === 'connecting'}
        >
          {status === 'connecting' ? 'Negotiating…' : 'Connect'}
        </button>
        <button
          className="px-4 py-2 rounded bg-slate-200 text-slate-800"
          onClick={() => {
            cleanup()
            setStatus('idle')
            setError(null)
          }}
        >
          Disconnect
        </button>
      </div>
    </section>
  )
}

async function waitForIceGathering(pc: RTCPeerConnection) {
  if (pc.iceGatheringState === 'complete') return
  await new Promise<void>((resolve) => {
    const checkState = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', checkState)
        resolve()
      }
    }
    pc.addEventListener('icegatheringstatechange', checkState)
  })
}

import React, { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  // Try MJPEG first (works well in <img>), fall back to the HTML stream_viewer in an iframe
  const safeTopic = encodeURI(ROS_CONFIG.topics.camera)
  const viewerBase = `${ROS_CONFIG.videoBase}/stream_viewer?topic=${safeTopic}`
  const [useViewer, setUseViewer] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [reloadToken, setReloadToken] = useState(0)

  const mjpegUrl = `${ROS_CONFIG.videoBase}/stream?topic=${safeTopic}&type=mjpeg&cacheBust=${reloadToken}`
  const viewerUrl = `${viewerBase}&cacheBust=${reloadToken}`

  const handleRetry = () => {
    setUseViewer(false)
    setError(null)
    setReloadToken((token) => token + 1)
  }

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Live Camera Feed</h2>
      <div className="flex flex-col gap-3">
        <div className="flex items-center justify-center min-h-64 bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 relative shadow-inner w-full">
          {!useViewer && (
            <img
              key={`mjpeg-${reloadToken}`}
              src={mjpegUrl}
              alt="HFH Robot Camera Feed"
              className="max-h-64 w-auto rounded object-contain"
              onError={(e) => {
                console.warn('MJPEG load failed, falling back to viewer', e);
                setUseViewer(true)
                setError('MJPEG stream unavailable, switching to HTML viewer…')
              }}
              onLoad={() => setError(null)}
            />
          )}

          {useViewer && (
            <iframe
              key={`viewer-${reloadToken}`}
              title="HFH Robot Camera Viewer"
              src={viewerUrl}
              className="w-full h-64 rounded border-0"
              onLoad={() => setError(null)}
            />
          )}
        </div>
        {error && (
          <div className="text-sm text-blue-800 bg-blue-100 border border-blue-200 rounded-lg p-3 flex flex-col gap-2">
            <div className="font-medium">Camera feed issue</div>
            <div>{error}</div>
            <div className="text-xs">Open the direct viewer: <a className="underline" href={viewerBase} target="_blank" rel="noreferrer">stream_viewer</a></div>
            <div>
              <button onClick={handleRetry} className="px-3 py-1 text-sm rounded bg-blue-600 text-white hover:bg-blue-700">Retry MJPEG</button>
            </div>
          </div>
        )}
      </div>
    </section>
  )
}

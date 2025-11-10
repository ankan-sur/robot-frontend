import { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  const safeTopic = encodeURIComponent(ROS_CONFIG.topics.camera)
  const mjpegUrl = `${ROS_CONFIG.videoBase}/stream?topic=${safeTopic}&type=mjpeg`
  const viewerUrl = `${ROS_CONFIG.videoBase}/stream_viewer?topic=${safeTopic}`
  const [useViewer, setUseViewer] = useState(false)
  const [error, setError] = useState<string | null>(null)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Camera Feed</h2>
      {error && (
        <div className="text-sm text-blue-800 bg-blue-100 border border-blue-200 rounded p-3 mb-3">
          {error}. Try the <a className="underline" href={viewerUrl} target="_blank" rel="noreferrer">stream viewer</a>.
        </div>
      )}
      <div className="flex items-center justify-center min-h-64 bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 relative shadow-inner">
        {!useViewer && (
          <img
            src={mjpegUrl}
            alt="HFH Robot Camera Feed"
            className="max-h-64 w-auto rounded object-contain"
            onError={() => {
              setUseViewer(true)
              setError('MJPEG stream unavailable, switching to HTML viewer…')
            }}
            onLoad={() => setError(null)}
          />
        )}
        {useViewer && (
          <iframe
            title="HFH Robot Camera Viewer"
            src={viewerUrl}
            className="w-full h-64 rounded border-0"
          />
        )}
      </div>
    </section>
  )
}

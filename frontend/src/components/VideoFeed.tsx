import { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  // Use encodeURI, not encodeURIComponent, so slashes in topic aren't escaped
  const safeTopic = encodeURI(ROS_CONFIG.topics.camera)
  const mjpegUrl = `${ROS_CONFIG.videoBase}/stream?topic=${safeTopic}&type=mjpeg`
  const [error, setError] = useState<string | null>(null)
  const [expanded, setExpanded] = useState(false)

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-lg font-semibold text-slate-800">Camera</h2>
        <button
          onClick={() => setExpanded(true)}
          className="px-2 py-1 text-xs rounded bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-300"
        >
          Expand
        </button>
      </div>
      {error && (
        <div className="text-sm text-slate-600 bg-slate-50 border border-slate-200 rounded p-2 mb-2">
          {error}
        </div>
      )}
      <div className="flex items-center justify-center bg-slate-900 rounded overflow-hidden aspect-video">
        <img
          src={mjpegUrl}
          alt="Camera Feed"
          className="w-full h-full object-contain"
          onError={() => setError('Stream unavailable')}
          onLoad={() => setError(null)}
        />
      </div>

      {/* Expanded modal */}
      {expanded && (
        <div className="fixed inset-0 z-50 bg-black/80 flex items-center justify-center p-4">
          <div className="bg-white rounded-lg shadow-2xl max-w-[95vw] w-full p-3">
            <div className="flex items-center justify-between mb-2">
              <div className="text-base font-semibold text-slate-800">Camera</div>
              <button
                onClick={() => setExpanded(false)}
                className="px-2 py-1 text-xs rounded bg-slate-100 hover:bg-slate-200 text-slate-700"
              >
                Close
              </button>
            </div>
            <div className="flex items-center justify-center bg-slate-900 rounded overflow-hidden">
              <img
                src={mjpegUrl}
                alt="Camera Feed"
                className="max-w-full max-h-[80vh] object-contain"
              />
            </div>
          </div>
        </div>
      )}
    </section>
  )
}
